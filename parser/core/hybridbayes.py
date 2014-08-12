'''Hybrid approach: Bayes conceptually, score functions in low-level.

Here's the process:
    - Build grammar (see grammar.py)

    - Apply the world (w) and robot (r) states into the Commands to get
        probabilities for which commands are most likely (P(C|w,r)).
        (Command.score).

    - Use all Commands to generate all Sentences (L).

    - Pre-match-score all Commands with Sentences based on Phrase
        matches (P(C|l)).

    - When an utterance comes in, score all Sentences based on Phrase
        matching strategies (subclass of MatchingStrategy) (estimate
        P(u|L); u space is infinite so we can't compute exactly).

    - Combine Command-Sentences scores with the Sentence-utterance
        scores to find the language score. (Command.lang_score) (i.e.
        use P(u|L) and marginalize L across P(L|C) to get P(C|u)).

    - Combine the w, r-induced probability P(C|w,r) with the language
        score P(C|u) to get the final probability P(C|u,w,r). Currently
        use language score first, then command score to break ties.
'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
from operator import attrgetter
import threading
import yaml

# Local
from constants import C, N
from grammar import CommandDict, Sentence, Command
from roslink import Robot, WorldObject, RobotCommand
from util import Error, Info, Debug, Numbers


########################################################################
# Classes
########################################################################

class Parser(object):
    '''This is where the magic happens.'''

    # Couple settings (currently for debugging)
    display_limit = 5

    def __init__(self, grammar_yaml=C.command_grammar):
        # Load
        self.command_dict = CommandDict(yaml.load(open(grammar_yaml)))

        # We can't be updating our guts while we try to churn something
        # out.
        self.lock = threading.Lock()

        # Initialize (for clarity)
        self.world_objects = None
        self.world_objects_for_generation = None
        self.robot = None
        self.phrases = None
        self.options = None
        self.templates = None
        self.commands = None

    def set_world(self, world_objects=None, robot=None):
        '''
        Updates the objects in the world and the robot.

        It will ONLY update parameters (world_objects, robot) that are
        not None. I.e. calling with (None, None) will have no effect
        (except re-scoring the internal grammar identically).

        Args:
            world_objects ([WorldObject], optional): Defaults to None.
            robot ([Robot], optional): Defaults to None.
        '''
        self.lock.acquire()
        if world_objects is not None:
            self.world_objects = world_objects
        if robot is not None:
            self.robot = robot
        if self.world_objects is not None and self.robot is not None:
            self._update_world_internal()
        self.lock.release()

    def parse(self, u):
        '''
        Args:
            u (str): utterance

        Returns:
            RobotCommand: The top command, or a clarification.
        '''
        self.lock.acquire()

        # Sanity check for state.
        if self.world_objects is None or self.robot is None:
            Error.p('Must set Parser world_objects and robot before parse().')
            return None

        # Translate utterance->Phrases and score all sentences.
        Info.p("Parser received utterance: " + u)
        u_sentence = Sentence([p for p in self.phrases if p.found_in(u)])
        Info.p('Utterance phrases: ' + str(u_sentence.get_phrases()))
        Sentence.compute_score(self.sentences, u_sentence)

        # Apply L.
        for c in self.commands:
            c.apply_l(self.sentences)
        Numbers.normalize(self.commands, 'lang_score')

        # Get top command (calculated by Command.cmp).
        self.commands.sort(cmp=Command.cmp)

        # If top two are indistinguishablly close, ask to clarify.
        first, second = self.commands[0], self.commands[1]
        # if (Numbers.are_floats_close(first.lang_score, second.lang_score) and
        #         Numbers.are_floats_close(first.score, second.score)):
        if (first.lang_score == second.lang_score and
                first.score == second.score):
            clarify_args = []
            # If same template, find non-shared properties to clarify.
            if first.template == second.template:
                for k, v in first.option_map.iteritems():
                    if second.option_map[k] != v:
                        clarify_args += [k]
            rc = RobotCommand.from_strs('clarify', clarify_args)
        else:
            # Otherwise, we got a solid top result.
            rc = RobotCommand.from_command(first)

        # We return a standard representation of the command.
        self._log_results(rc)
        self.lock.release()
        return rc

    def _log_results(self, rc):
        '''
        Write results of parse to log.

        Args:
            rc (RobotCommand): What we're returning.
        '''
        if Info.printing:
            # Display sentences.
            self.sentences.sort(key=attrgetter('score'), reverse=True)
            topscore = self.sentences[0].score
            Info.p('Top sentences:')
            for s in self.sentences:
                if s.score == topscore:
                    Info.pl(1, s)
                else:
                    break

            # Display commands.
            top_lscore = self.commands[0].lang_score
            top_cscore = self.commands[0].score
            Info.p("Top commands:")
            for c in self.commands:
                if c.lang_score == top_lscore and c.score == top_cscore:
                    Info.pl(1, c)
                else:
                    break

            # For clarity, show what we're returning.
            Info.p('Returning command: %s' % (str(rc)))

    def _update_world_internal(self):
        '''
        Re-generates all phrases, options, parameters, templates,
        commands, sentences based on (presumably) updated world objects
        and/or robot state.

        The following must be set prior to calling:
            - self.world_objects ([WorldObject])
            - self.robot (Robot)
        '''
        self._update_world_internal_maybe_generate()
        self._update_world_internal_score()

    def _update_world_internal_maybe_generate(self):
        '''
        This part generates all templates (phrases, options, commands,
        sentences) and takes a long time. It doesn't apply the world
        objects or robot to the prior scores. This only needs to happen
        when the world objects change in a significant way (i.e. they
        are different objects, not just differently reachable). This is
        because the sentences that the same objects generate won't
        change if only their reachability and location, not their
        properties, have changed.
        '''
        # First, see whether we need to do this at all.
        if WorldObject.check_objects_match(
                self.world_objects, self.world_objects_for_generation):
            Debug.p("Objects match; not re-generating.")
            return
        else:
            Debug.p("Objects do not match; re-generating.")

        # Make templates (this extracts options and params).
        self.world_objects_for_generation = self.world_objects
        self.phrases, self.options, self.templates = (
            self.command_dict.get_grammar(self.world_objects))

        # Some initial displaying
        Info.p("Phrases: " + str(len(self.phrases)))
        Info.p("Options: " + str(len(self.options)))
        Debug.p('Templates:')
        for t in self.templates:
            Debug.pl(1, t)
        Info.p("Templates: " + str(len(self.templates)))

        # Make commands
        self.commands = [ct.generate_commands() for ct in self.templates]
        self.commands = [i for s in self.commands for i in s]  # Flatten.
        Info.p("Commands: " + str(len(self.commands)))

        # Make sentences
        self.sentences = [c.generate_sentences() for c in self.commands]
        self.sentences = [i for s in self.sentences for i in s]  # Flatten.
        Info.p("Sentences: " + str(len(self.sentences)))

        # Pre-score commands with all possible sentences.
        for c in self.commands:
            c.score_match_sentences(self.sentences)  # Auto-normalizes.

    def _update_world_internal_score(self):
        '''
        This part applies the world objects and robot to the score. It
        is relatively fast. This alone can be called if the world
        objects are identicial in core properties.
        '''
        # Apply W and R to weight C prior.
        for c in self.commands:
            c.apply_w()
            c.apply_r(self.robot)
        Numbers.normalize(self.commands, min_score=N.MIN_SCORE)

        # Display commands.
        if Debug.printing:
            self.commands.sort(key=attrgetter('score'), reverse=True)
            Debug.p(
                "Top %d commands (before utterance):" % (Parser.display_limit))
            for idx, c in enumerate(self.commands):
                if idx < Parser.display_limit:
                    Debug.pl(1, c)
            Debug.p(
                'Total (%d): %0.2f' % (
                    len(self.commands),
                    sum([c.score for c in self.commands])
                )
            )
