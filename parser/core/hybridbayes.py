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
from collections import Counter
from operator import attrgetter
import time
import threading
import yaml

# Local
from constants import C, N
from grammar import CommandDict, Sentence, Command, ObjectOption
from roslink import Robot, WorldObject, RobotCommand
from util import Error, Warn, Info, Debug, Numbers


# ######################################################################
# Constants
# ######################################################################

# How close command scores can be before they are considered equal.
CSCORE_EPSILON = 0.00001

# Smoothing over grounding; objects with no matching phrase in the
# referring expression get this score. This should be balanced with
# the numbers in matchers.py. Note that this is a score and not a
# probability.
GROUND_BASE_SCORE = 0.2


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
        self.robot = None
        self.phrases = None
        self.options = None
        self.templates = None
        self.commands = None

    ####################################################################
    # API
    ####################################################################

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

    def describe(self):
        '''
        Describes the current world objects with a policy.

        The policy is currently hard-coded, but could be parameterized,
        extended, etc.

        The policy is to use the least words possible to uniquely
        identify an object with a description. In deciding which words
        to use, we have a priority ordering over adjectives that is
        motivated by the literature.

        Returns:
            {str: str}: Map of object names to their description.
        '''
        self.lock.acquire()
        descs = {}
        obj_opts = [o for o in self.options if isinstance(o, ObjectOption)]
        Info.p(obj_opts)

        # Get flattened list of identifiers (color & shape) & count
        # occurrences of each.
        idents = Counter([
            i for s in [
                o.structured_word_options[ObjectOption.IDENT] +
                o.structured_word_options[ObjectOption.TYPE]
                for o in obj_opts
            ]
            for i in s])

        for opt in obj_opts:
            desc = []

            # Extract to avoid long variable names.
            swo = opt.structured_word_options
            starts, uniques, ident, type_ = (
                swo[ObjectOption.START],
                swo[ObjectOption.UNIQUE],
                swo[ObjectOption.IDENT][0],  # Only 1.
                swo[ObjectOption.TYPE][0]  # Only 1.
            )
            unique_names = [u.name for u in uniques]

            # Debug
            Info.pl(0, opt)
            Info.pl(1, 'swo: ' + str(swo))
            Info.pl(1, 'starts: ' + str(starts))
            Info.pl(1, 'uniques: ' + str(uniques))
            Info.pl(1, 'ident: ' + str(ident))
            Info.pl(1, 'type: ' + str(type_))

            # First add starters.
            desc = starts[:]  # Don't want to modify swo.

            # See whether type is sufficient.
            if idents[type_] > 1:
                # See if there are any adjectives with higher priority
                # than color.
                use_color = True
                for top_adj in C.color_priority:
                    if top_adj in unique_names:
                        use_color = False

                # If nothing higher priority than color, and color is
                # identifying, then use it.
                if use_color and idents[ident] == 1:
                    desc += [ident]
                else:
                    # Color's not unique or of lower priority; pull from
                    # the unique list.
                    # NOTE(mbforbes): Currently just pull first off of
                    # the list. Change the order by chaning C.m_wo.
                    if len(uniques) > 0:
                        desc += [uniques[0]]
                    # Note that if we don't have any uniques, then we
                    # have objects that we cannot distinguish. This will
                    # happen when we have enough objects.

            # Always add type at end.
            desc += [type_]
            Info.pl(1, 'result: ' + str(desc))
            descs[opt.name] = ' '.join(
                [str(wo.get_phrases()[0][0]) for wo in desc])

        Info.pl(0, 'returning: ' + str(descs))
        self.lock.release()
        return descs

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

        # See how many results we got that are top ranked.
        first_cmd = self.commands[0]
        top_lscore = first_cmd.lang_score
        top_cscore = first_cmd.score
        top_cmds = [
            c for c in self.commands if
            Numbers.are_floats_close(c.lang_score, top_lscore) and
            Numbers.are_floats_close(c.score, top_cscore, CSCORE_EPSILON)]
        if len(top_cmds) == 1:
            # One top command; return it.
            rc = RobotCommand.from_command(first_cmd, u_sentence, u)
        else:
            # Multiple top commands; ask to clarify.
            # See if we can be more specific about clarifying.
            rc = self._get_clarify_rc(top_cmds, u)

        # We return a standard representation of the command.
        self._log_results(rc)
        self.lock.release()
        return rc

    def ground(self, gq):
        '''
        Args:
            gq (str): Grounding query.

        Returns:
            {str: float}: Map of obj : P(obj).
        '''
        self.lock.acquire()

        res = {}
        gq_sentence = Sentence([p for p in self.phrases if p.found_in(gq)])
        opts = [o for o in self.options if isinstance(o, ObjectOption)]

        # Check if we don't have any objects (actually quite common).
        if len(opts) == 0:
            Warn.p("Trying to do grounding with no objects; empty result.")
            self.lock.release()
            return res

        scores = []
        for o in opts:
            phrase_sets = o.get_phrases()
            sentences = [Sentence(phrases) for phrases in phrase_sets]
            # Match with grounding scores for phrases.
            Sentence.compute_score(
                sentences,
                gq_sentence,
                normalize=False,
                ground=True
            )
            best_score = max([s.score for s in sentences])
            scores += [best_score]

        # Normalize to valid probability distribution and save.
        scores = Numbers.normalize_list(scores, GROUND_BASE_SCORE)
        for i in range(len(scores)):
            res[opts[i].name] = scores[i]

        # Log for convenience
        Info.p("Grounding for query: " + gq)
        for obj, prob in res.iteritems():
            Info.pl(1, obj + ": " + str(prob))

        self.lock.release()
        return res

    def _get_clarify_rc(self, top_cmds, u):
        '''
        Gets robot command to ask for clarification that is as helpful
        as possible.

        Prereq: multiple top commands (all equally scoring in lang and
        score).

        Args:
            top_cmds ([Command]): Subset of self.commands.
            u (str): Utterance: what we heard the user say.

        Returns:
            RobotCommand: 'Clarify' command, with some number of args,
                which are options to clarify. If templates don't match,
                then no args are returned.
        '''
        # Check for matching template (i.e. verb).
        first_template = top_cmds[0].template
        for cmd in top_cmds:
            if cmd.template != first_template:
                # Doesn't match; we need to clarify the basic command.
                return RobotCommand.from_strs('clarify', [], [], u)

        # If we made it here, all top commands have the same template.
        # Thus, we can look for options to clarify. n^3 computation.
        clarify_args = set()
        for cmd1 in top_cmds:
            for cmd2 in top_cmds:
                for opt_name, opt_val in cmd1.option_map.iteritems():
                    if cmd2.option_map[opt_name] != opt_val:
                        clarify_args.add(opt_name)
        return RobotCommand.from_strs('clarify', list(clarify_args), [], u)

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
            for c in self.commands[:10]:
                Info.pl(1, c)
                # if c.lang_score == top_lscore and c.score == top_cscore:
                #     Info.pl(1, c)
                # else:
                #     break

            # For clarity, show what we're returning.
            Info.p('Returning command: %s' % (str(rc)))
            Info.p('.... with phrases: %s' % (' '.join(rc.phrases)))

    def _update_world_internal(self):
        '''
        Re-generates all phrases, options, parameters, templates,
        commands, sentences based on (presumably) updated world objects
        and/or robot state.

        The following must be set prior to calling:
            - self.world_objects ([WorldObject])
            - self.robot (Robot)
        '''
        self._update_world_internal_generate()
        self._update_world_internal_score()

    def _update_world_internal_generate(self):
        '''
        This part generates all templates (phrases, options, commands,
        sentences) and takes a long time. It doesn't apply the world
        objects or robot to the prior scores.
        '''
        # Timing
        # Time the generation, as it probably isn't woth the
        # optimization if it gives us object mismatch bugs we have to
        # much about and solve.
        times = []
        times += [(time.time(), "start")]

        # Make templates (this extracts options and params).
        self.phrases, self.options, self.templates = (
            self.command_dict.get_grammar(self.world_objects))

        # Timing
        gitems = len(self.phrases) + len(self.options) + len(self.templates)
        times += [(time.time(), "get grammar (%d)" % (gitems))]

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

        # Timing
        times += [(time.time(), "make commands (%d)" % (len(self.commands)))]

        # Make sentences
        self.sentences = [c.generate_sentences() for c in self.commands]
        self.sentences = [i for s in self.sentences for i in s]  # Flatten.
        Info.p("Sentences: " + str(len(self.sentences)))

        # Timing
        times += [(time.time(), "make sentences (%d)" % (len(self.sentences)))]

        # Pre-score commands with all possible sentences.
        for c in self.commands:
            c.score_match_sentences(self.sentences)  # Auto-normalizes.

        # Timing
        cxs = len(self.commands) * len(self.sentences)
        times += [
            (time.time(), "score match commands w/ sentences (%d)" % (cxs))]
        self._display_timing(times)

    def _display_timing(self, tuples):
        '''
        Display timing info.

        Args:
            tuples ([(float, str)])
        '''
        Info.p("Timing:")
        start_time = tuples[0][0]
        last_time = start_time
        for t, n in tuples[1:]:
            diff = t - last_time
            Info.pl(1, "%0.4f %s" % (diff, n))
            last_time = t
        Info.pl(1, "%0.4f %s" % (last_time - start_time, 'total'))

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
