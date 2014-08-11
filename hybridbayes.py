'''Hybrid approach: Bayes conceptually, score functions in low-level.

Here's the process:
    - Extract all Options, which contain Phrases
        - WordOptions' phrases come from a template (COMMAND_GRAMMAR)
        - ObjectOptions' phrases come from the object properties
            (objects are loaded into WorldObjects)

    - Use the Options to build Parameters

    - Use the Paramters to build CommandTemplates

    - Use the CommandTemplates to generate all Commands

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
import code
from collections import OrderedDict, defaultdict
import copy
from operator import attrgetter
import pprint
import sys
import threading
import yaml

# Local
from util import Logger, Error, Info, Debug, Numbers, Algo
from matchers import DefaultMatcher, MatchingStrategy, Matchers


########################################################################
# Module-level constants
########################################################################

# Global options
COMMAND_GRAMMAR = 'commands.yml'
WORLD = 'world.yml'
OBJ_PARAM = 'obj'

# How to index into object properties by side
SIDES = yaml.load(open(COMMAND_GRAMMAR))['parameters']['side']
sides_map = {}
for idx, side_name in enumerate(SIDES):
    sides_map[side_name] = idx
S = sides_map

# Map (M) from component options to object properties (OP).
M_OP = {
    'above': 'is_above_reachable',
    'next_to': 'is_nextto_reachable',
    'to_left_of': 'is_leftof_reachable',
    'to_right_of': 'is_rightof_reachable',
    'in_front_of': 'is_frontof_reachable',
    'behind': 'is_behind_reachable',
    'on_top_of': 'is_topof_reachable',
    'near': 'is_near_reachable',
}

# Map (M) from component options to robot properties (RP).
M_RP = {
    'up': 'can_move_up',
    'down': 'can_move_down',
    'to_left': 'can_move_toleft',
    'to_right': 'can_move_toright',
    'forward': 'can_move_forward',
    'backward': 'can_move_backward',
}

# Map (M) from object properties to WordOption names (WO)
M_WO = {
    # Location
    'is_leftmost': 'left_most',
    'is_rightmost': 'right_most',
    'is_farthest': 'farthest',
    'is_nearest': 'nearest',
    # Size
    'is_biggest': 'biggest',
    'is_smallest': 'smallest',
}
M_WO_INVERSE = {v: k for k, v in M_WO.iteritems()}

# Command score
START_SCORE = 1.0  # To begin with. Maybe doesn't matter.
MIN_SCORE = 0.1  # What to boost scores to *before* normalizing.
P_LOCUNR = -5.0  # Penalty: the requested location is unreachable.
P_OBJUNR = -5.0  # Penalty: the requested object cannot be picked up.
P_NOTLASTSIDE = -0.1  # Penalty: the side wasn't the last commanded.
P_GSTATE = -8.0  # Penalty: nonsensical gripper state change requested.
P_DUMBSTOP = -10.0  # Penalty: can't stop when not executing.
P_DUMBEXEC = -10.0  # Penalty: can't execute when already doing so.
P_BADPP = -5.0  # Penalty: bad pickup/place command requested (from GS)

# How close language scores have to be before we start to use the
# command score (world and robot-influenced) to break ties. This really
# needs to be tuned as the number of comamnds (sentences) grows.
LANGUAGE_TIE_EPSILON = 0.001


########################################################################
# Classes
########################################################################

class CommandDict(object):
    '''The Python representation of our YAML-defined commands file.'''

    longest_cmd_name_len = 0

    def __init__(self, ydict):
        '''
        Args:
            ydict (dict): YAML-loaded dictionary.
        '''
        self.ydict = ydict
        CommandDict.longest_cmd_name_len = max(
            [len(cmd) for cmd, params in self.ydict['commands'].iteritems()])

        # Make all adjectives in object option descriptors optional.
        for desc_type, props in ydict['descriptors'].iteritems():
            if 'adjective' in props and props['adjective']:
                for option in props['options']:
                    ydict['options'][option]['optional'] = True

    def get_grammar(self, wobjs):
        '''
        Args:
            wobjs ([WorldObject]): The object we see in the world (or
                from a YAML file).

        Returns:
            3-tuple of: (
                [Phrase]
                [Option]
                [CommandTemplate],

            )
        '''
        phrase_map = self._make_phrases()
        options = self._make_options(phrase_map, wobjs)
        parameters = self._make_parameters(options)
        templates = self._make_templates(parameters)
        return (phrase_map.values(), options.values(), templates)

    def _make_templates(self, parameters):
        '''
        Args:
            parameters ([{str: Parameter}])

        Returns:
            [CommandTemplate]
        '''

        templates = []
        for cmd, pnames in self.ydict['commands'].iteritems():
            params = []
            for pname in pnames:
                # We omit parameters that don't exist (like objects in
                # the world), so check for this here.
                if pname in parameters:
                    params += [parameters[pname]]
                else:
                    break

            # If any parameters were missing, don't add.
            if len(params) == len(pnames):
                templates += [CommandTemplate(cmd, params)]
        return templates

    def _make_parameters(self, options):
        '''
        Args:
            options ({str: Option})

        Returns:
            {str: Parameter}
        '''
        params = {}

        # First, find all possible parameter names.
        param_set = set()
        for cmd, pnames in self.ydict['commands'].iteritems():
            for pname in pnames:
                param_set.add(pname)

        # Then, make a complete mapping of them.
        pdict = self.ydict['parameters']
        for pname in list(param_set):
            if pname == OBJ_PARAM:
                opts = [
                    opt for name, opt in options.iteritems()
                    if isinstance(opt, ObjectOption)]
                # Filter out parameter if no objects.
                if len(opts) == 0:
                    continue
            elif pname in pdict:
                # We have an actual entry, which means the parameter has
                # multiple options.
                opt_names = pdict[pname]
                opts = [options[opt_name] for opt_name in opt_names]
            else:
                # No entry; assume 1:1 mappingto opt_name. This errors
                # if there wasn't an option mapping, which is good.
                opts = [options[pname]]
            params[pname] = Parameter(pname, opts)

        Debug.p('Params: ' + str(params.values()))
        return params

    def _make_options(self, phrase_map, wobjs):
        '''
        Args:
            phrase_map ({(str, class): Phrase}): Map of
                {('words', strategy): Phrase}.
            wobjs ([WorldObject]): The object wes see in the world (or
                from a YAML file or python dict).

        Returns:
            {str: Option}: Map of {'option name': Option}.
        '''
        options = {}
        # Make word options from phrases.
        for opt_name, props in self.ydict['options'].iteritems():
            # Get strategy, if special.
            matcher = Matchers.MATCHERS['default']
            if 'strategy' in props:
                matcher = Matchers.MATCHERS[props['strategy']]

            # Get phrases
            phrases = [
                phrase_map[(words, matcher)] for words in props['phrases']]

            # Make our word option and add it.
            w_opt = WordOption(opt_name, phrases, props)
            options[opt_name] = w_opt

        # Now make object options, because they are described by word
        # options.
        for wobj in wobjs:
            options[wobj.get_property('name')] = ObjectOption(wobj, options)

        Debug.p('Options: ' + str(options.values()))
        return options

    def _make_phrases(self):
        '''
        Returns:
            {str: Phrase}: Map of {'words': Phrase}.
        '''
        phrases = {}
        # Make phrases
        for opt_name, props in self.ydict['options'].iteritems():
            # Get strategy, if special.
            matcher = Matchers.MATCHERS['default']
            if 'strategy' in props:
                matcher = Matchers.MATCHERS[props['strategy']]

            # Get phrases
            for words in props['phrases']:
                key = (words, matcher)
                # Avoid making duplicates and pushing the other ones
                # out just for convenience.
                if key not in phrases:
                    phrases[key] = Phrase(words, matcher)

        Debug.p('Phrases: ' + str(phrases.values()))
        return phrases


class CommandTemplate(object):
    '''An uninstantiated command.'''

    def __init__(self, name, params):
        '''
        Args:
            name (str)
            params ([Parameter])
        '''

        self.name = name
        self.params = params

    def __repr__(self):
        arr = ['<<' + self.name + '>>:'] + [str(p) for p in self.params]
        return ' '.join(arr)

    def generate_commands(self):
        '''Generates all possible commands specified by this template.

        Simply enumerates all possible options for all parameters.

        Returns:
            [Command]
        '''
        param_copy = self.params[:]  # Just copy all obj. references.
        opt_maps = CommandTemplate._gen_opts(param_copy)
        return [Command(self.name, om, self) for om in opt_maps]

    def has_params(self, pnames):
        '''
        Returns whether this template has ALL parameters specified by
        name in pnames.

        Args:
            pnames ([str])

        Returns:
            bool
        '''
        pnames = list(set(pnames))  # Avoid duplicate problems
        for param in self.params:
            if param.name in pnames:
                pnames.remove(param.name)
        return len(pnames) == 0

    @staticmethod
    def _gen_opts(todo, results=[]):
        '''
        Args:
            todo [Parameter]
            results ([{str: Option}])

        Returns:
            [{str: Option}]
        '''
        if len(todo) == 0:
            return results
        param = todo.pop(0)
        next_opts = param.get_options()
        # Debug.p("Processing options: " + str(next_opts))
        if results == []:
            new_results = []
            for opt in next_opts:
                entry = OrderedDict()
                entry[param.name] = opt
                new_results += [entry]
        else:
            new_results = []
            for opt in next_opts:
                for r in results:
                    newr = copy.copy(r)
                    newr[param.name] = opt
                    new_results += [newr]
        return CommandTemplate._gen_opts(todo, new_results)


class Command(object):
    '''A fully-instantiated command.

    Has state: YES
    '''

    def __init__(self, name, option_map, template):
        '''
        Args:
            name (str)
            option_map ({str: Option})
            template (CommandTemplate)
        '''
        self.name = name
        self.option_map = option_map
        self.template = template
        self.phrase_sets = []  # Caching
        self.sentences = []  # Caching

        # P(C|W,R)
        self.score = START_SCORE

        # P(L|C)
        self.sentence_match_probs = defaultdict(float)  # Default: 0.0.

        # P(L|C) * P(L) (marginalize across L)
        self.lang_score = 0.0

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return "%s  %s  %s" % (
            self._score_str(), self._name_str(), self._opt_str())

    def pure_str(self):
        '''
        Returns a string without any score info.

        Returns:
            str
        '''
        return "%s  %s" % (self._name_str(), self._opt_str())

    def opt_str_list(self):
        '''
        Returns a list of strings, one for each option, of the option's
        'cannonical' name.
        '''
        return [v.pure_str() for k, v in self.option_map.iteritems()]

    def get_name(self):
        '''
        Returns the 'cannonical' name of the command (no padding).
        '''
        return self.name

    def _score_str(self):
        '''
        Any scores that have been set.

        Returns:
            str
        '''
        score_str = "score: %0.4f" % (self.score)
        score_str += ", lang_score: %0.4f" % (self.lang_score)
        return score_str

    def _name_str(self):
        '''
        Name plus padding.

        Returns:
            str
        '''
        lnl = CommandDict.longest_cmd_name_len
        padding = 0 if len(self.name) == lnl else lnl - len(self.name)
        return ''.join(['<', self.name, '>', ' ' * padding])

    def _opt_str(self):
        '''
        Option list in readable format.

        Returns:
            str
        '''
        return ', '.join([': '.join(
            [str(k), str(v)]) for k, v in self.option_map.iteritems()])

    def generate_sentences(self):
        '''
        Returns:
            [Sentence]
        '''
        # Cache.
        if len(self.sentences) == 0:
            phrase_lists = Algo.gen_phrases(self.option_map.values())
            self.sentences = [Sentence(pl) for pl in phrase_lists]
        return self.sentences

    def has_obj_opt(self):
        '''
        Returns:
            bool
        '''
        return len(self.get_objs()) > 0

    def get_objs(self):
        '''
        Returns:
            [WorldObject]
        '''
        return [
            opt.get_wobj() for pname, opt in self.option_map.iteritems()
            if isinstance(opt, ObjectOption)]

    def get_phrase_sets(self):
        '''
        Returns:
            [[Phrase]]: The length is the number of options this command
                has, and each element is a list that contains all of
                that options' phrases.
        '''
        # We cache and only compute if it's never been computed. This
        # only happens once currently, but it shouldn't be done twice.
        if len(self.phrase_sets) == 0:
            phrase_sets = []
            for pname, opt in self.option_map.iteritems():
                opt_phrase_sets = opt.get_phrases()
                for phrase in opt_phrase_sets:
                    phrase_sets += [phrase]
            self.phrase_sets = phrase_sets
        return self.phrase_sets

    def apply_w(self):
        '''
        Applies world (objects) influence to command scores.

        Negatively affect commands that contain objects referring to
        'impossible configurations.'

        - lookat: obj exist?
        - move_rel: move obj to relpos w/ side possible?
        - place_rel: same as move_rel
        - pickup: pickup obj w/ side possible?
        '''
        if not self.has_obj_opt():
            return

        # Currently, there should be exactly one object per command.
        wobj = self.get_objs()[0]

        # We'll filter by name, as template sets will be brittle.
        # cmds: move_rel, place (params: relpos, side)
        if self.name in ['move_rel', 'place']:
            relpos = self.option_map['rel_pos'].name
            side = self.option_map['side'].name
            # NOTE(mbforbes): What do we assume if there's no such
            # property? I'll assume no informations means do not affect
            # the score.
            if wobj.has_property(M_OP[relpos]):
                reachables = wobj.get_property(M_OP[relpos])
                if side in SIDES:
                    loc_reachable = reachables[S[side]]
                else:
                    # Side == both
                    loc_reachable = reachables[0] and reachables[1]
                if not loc_reachable:
                    self.score += P_LOCUNR

        # cmds: pickup (params: side)
        if self.name in ['pick_up']:
            side = self.option_map['side'].name
            # NOTE(mbforbes): What do we assume if there's no such
            # property? I'll assume no informations means do not affect
            # the score.
            if wobj.has_property('is_pickupable'):
                reachables = wobj.get_property('is_pickupable')
                if side in SIDES:
                    loc_reachable = reachables[S[side]]
                else:
                    # Side == both
                    loc_reachable = reachables[0] and reachables[1]
                if not loc_reachable:
                    self.score += P_OBJUNR

    def apply_r(self, robot):
        '''
        Applies robot (state) influence to command scores.

        Negatively affect commands that contain nonsensical operations
        (like open right hand with right hand open) or non-last-referred
        side commands (like move right hand when left hand last moved).

        - all side: last commanded? (less linkely down)
        - all open/close: based on state (impossible down)
        - all pickup/place: based on state (impossible down)
        - move_abs: absdir possible to move to

        Args:
            commands ([Command])
            robot (Robot)
        '''
        if robot.has_property('is_executing'):
            is_exec = robot.get_property('is_executing')
            if self.name == 'stop' and not is_exec:
                self.score += P_DUMBSTOP
            elif self.name == 'execute' and is_exec:
                self.score += P_DUMBEXEC

        # All following robot applications involve a side.
        if not self.template.has_params(['side']):
            return

        cmd_side = self.option_map['side'].name
        side_idx = S[cmd_side]

        # Side: last moved? Less likely down.
        if robot.has_property('last_cmd_side'):
            last_cmd_side = robot.get_property('last_cmd_side')
            if (last_cmd_side != 'neither' and
                    self.option_map['side'].name != last_cmd_side):
                self.score += P_NOTLASTSIDE

        # Open/closed & pickup/place: Basd on gripper state (impossible
        # configurations have scores lowered).
        if robot.has_property('gripper_states'):
            gs = robot.get_property('gripper_states')[side_idx]
            if self.name == 'open':
                if gs == 'open':
                    self.score += P_GSTATE
            elif self.name == 'close':
                if gs == 'closed_empty' or gs == 'has_obj':
                    self.score += P_GSTATE
            elif self.name == 'pick_up':
                if gs == 'has_obj':
                    self.score += P_BADPP
            elif self.name == 'place':
                if gs == 'open' or gs == 'closed_empty':
                    self.score += P_BADPP

        # move_abs: absdir possible to move to
        if self.name == 'move_abs':
            prop_name = M_RP[self.option_map['abs_dir'].name]
            if robot.has_property(prop_name):
                loc_reachable = robot.get_property(prop_name)[side_idx]
                if not loc_reachable:
                    self.score += P_LOCUNR

    def score_match_sentences(self, sentences):
        '''
        Score how well a command matches with sentences.

        This can be done in advance of actually receiving an utterance.
        The result is stored in self.sentence_match_probs

        Args:
            sentences ([Sentence])
        '''
        s_score = 1.0 / len(self.sentences)
        for s in self.sentences:
            self.sentence_match_probs[s] = s_score

    def apply_l(self, sentences):
        '''
        Applies utterance-scored sentences to command-matched score.

        Must first call score_match_sentences(...).

        Args:
            sentences ([Sentence])
        '''
        self.lang_score = 0.0
        # Only its own sentences have any score, so just iterate over
        # them.
        for s in self.sentences:
            self.lang_score += s.score * self.sentence_match_probs[s]
        return

    def cmp(cmd1, cmd2):
        '''
        Compares commands for sorting. This is the replacement for the
        'get_final_p' function and 'final_p' attribute. We do this
        because how we rank commands is meaningful relative to
        each-other, but hard to do in an absolute way.

        Implementation-specific details are varying greatly during the
        course of this project, so see the code for details.

        Args:
            cmd1 (Command)
            cmd2 (Command)

        Returns:
            int: n such that
                n < 0 if cmd1 comes before cmd2
                n == 0 if cmd1 comes at the same place as cmd2
                n > 0 if cmd2 comes before cmd1
        '''
        if Numbers.are_floats_close(
                cmd1.lang_score, cmd2.lang_score, LANGUAGE_TIE_EPSILON):
            diff = cmd2.score - cmd1.score
        else:
            diff = cmd2.lang_score - cmd1.lang_score
        if diff == 0:
            return 0
        else:
            return -1 if diff < 0 else 1


class Parameter(object):
    '''A class that holds parameter info, including possible options.'''

    def __init__(self, name, options):
        self.name = name
        self.options = options

    def get_options(self):
        return self.options

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ''.join(['<', self.name, '>'])


class Option(object):
    '''Interface for options.'''

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ''.join(['|', self.name, '|'])

    def get_phrases(self):
        '''
        Returns:
            [[Phrase]]
        '''
        Error.p("Option:get_phrases must be implemented by a subclass.")
        sys.exit(1)

    def pure_str(self):
        '''
        Returns the 'cannonical' name of the option.

        Returns:
            str
        '''
        return self.name

    def is_optional(self):
        '''
        ObjectOptions are not optional (though WordOptions they contain
        may be).

        Returns:
            bool
        '''
        return self.opt


class WordOption(Option):
    '''Holds options info, including all possible referring phrases.'''

    def __init__(self, name, phrases, props):
        '''
        Args:
            name (str)
            phrases ([Phrases])
            props ({})
        '''
        self.name = name
        self.phrases = phrases
        self.opt = 'optional' in props and props['optional']

    def get_phrases(self):
        '''
        Returns a list, where each element is a single-element list of
        a possible phrase.

        Returns:
            [[Phrase]]
        '''
        return [[p] for p in self.phrases]


class ObjectOption(Option):
    '''Holds options info for object, so can generate all possible
    referring phrases.'''

    def __init__(self, world_obj, options):
        '''
        Args:
            world_obj (WorldObject)
            options ({str: Option}): Map of option name: existing
                Option. Will contain all WordOptions as well as the
                ObjectOptions that have been created so far. What we
                care about are the WordOptions (any inter-object state
                is pre-computed before objects are sent).
        '''
        # self.param_name = OBJ_PARAM
        self.name = world_obj.get_property('name')
        self.world_obj = world_obj
        self.word_options = self._get_word_options(options)
        self.phrases = []  # Compute later and cache.
        self.phrases_skipping = []  # Compute later and cache.
        self.opt = False

    def _get_word_options(self, options):
        '''
        Args:
            options ({str: Option}): Map of option name: existing
                Option. Will contain all WordOptions as well as the
                ObjectOptions that have been created so far. What we
                care about are the WordOptions (any inter-object state
                is pre-computed before objects are sent).

        Returns:
            [WordOption]: The word options that describe this.
        '''
        # Here is the ordering use for word options (phrases can appear
        # in any order for utterances, so this only matters for when
        # we generate phrases to be spoken to the user).
        #
        # Further, note that we don't currently weight estimates for
        # object properties, as they are bools. If we do weight them,
        # we'll want to use this to decide which properties to output
        # (talk about color rather than location if we're the most sure
        # about it).
        #
        # - location (leftmost, rightmost, etc.)
        # - size (biggest, smallest)
        # - color (red, green, blue)
        # - type (box, cup, unknown)
        #
        # First, we add the essential starter ('the').
        word_options = [options['the']]

        # Next, we do the unique properties (left-most, biggest, etc.)
        for object_property, word_option_name in M_WO.iteritems():
            # NOTE: The properties are currently bools, so
            # get_property(...) is enough.
            if (self.world_obj.has_property(object_property) and
                    self.world_obj.get_property(object_property)):
                word_option = options[word_option_name]
                word_options += [word_option]

        # Now we do non-unique properties (red, box, etc.)
        props = ['color', 'type']
        for prop in props:
            if self.world_obj.has_property(prop):
                prop_val = self.world_obj.get_property(prop)
                word_option = options[prop_val]
                word_options += [word_option]

        return word_options

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ''.join(['|', self.name, '|'])

    def get_wobj(self):
        '''
        Returns:
            WorldObject
        '''
        return self.world_obj

    def get_phrases(self, skipping=False):
        '''
        Returns a list of possible phrases, where each element is a list
        of Phrases.

        Returns:
            [[Phrase]]
        '''
        if skipping:
            if len(self.phrases_skipping) == 0:
                self.phrases_skipping = Algo.gen_phrases(
                    self.word_options, True)
            return self.phrases_skipping
        else:
            if len(self.phrases) == 0:
                self.phrases = Algo.gen_phrases(self.word_options, False)
            return self.phrases


class Sentence(object):
    '''Our representation of a perfect 'utterance'.

    A series of phrases.

    Has state: YES
    '''

    def __init__(self, phrases):
        '''
        Args:
            phrases ([Phrase])
        '''
        self.phrases = phrases
        self.score = 0

    def __repr__(self):
        '''
        Returns:
            str
        '''
        phrases = ' '.join(["'" + str(p) + "'" for p in self.phrases])
        return "%0.6f  %s" % (self.score, phrases)

    def __eq__(self, other):
        '''
        Args:
            other (Sentence)

        Returns:
            bool
        '''
        # Allow mismatched ordering.
        for phrase in self.phrases:
            if phrase not in other.get_phrases():
                return False
        return True

    def __ne__(self, other):
        '''
        Args:
            other (Sentence)

        Returns:
            bool
        '''
        return not self.__eq__(other)

    def get_raw(self):
        '''
        Returns just the sentences words as a string

        Returns:
            str
        '''
        return ' '.join(str(p) for p in self.phrases)

    def get_phrases(self):
        '''
        Returns:
            [Phrase]
        '''
        return self.phrases

    @staticmethod
    def compute_score(sentences, u_sentence):
        '''
        Args:
            sentences ([Sentence]): Sentences to score.
            u_sentence (Sentence): The utterance as a Sentence.
        '''
        u_phrases = u_sentence.get_phrases()

        # Mark phrases as seen.
        for p in u_phrases:
            p.seen = True

        # Score all sentences by adding scores of seen phrases.
        for sentence in sentences:
            sentence.score = 0.0
            for phrase in sentence.phrases:
                if phrase.seen:
                    sentence.score += phrase.get_score()

        # Unmark
        for p in u_phrases:
            p.seen = False

        # Normalize
        Numbers.make_prob(sentences)


class Phrase(object):
    '''Holds a set of words and a matching strategy for determining if
    it is matched in an utterance.

    Has state: only briefly, and only during
    Sentence::compute_score(...).
    '''

    def __init__(self, words, strategy):
        '''
        Args:
            words ([str])
            strategy (MatchingStrategy)
        '''
        self.words = words
        self.strategy = strategy
        self.score = strategy.score()

        # Mark when seen by an utterance.
        self.seen = False

    def get_score(self):
        '''
        Returns:
            float
        '''
        return self.score

    def found_in(self, utterance):
        '''
        Args:
            utterance (str)

        Returns:
            bool
        '''
        return self.strategy.match(self.words, utterance)

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return self.words


class WorldObject(object):
    '''
    Contains data about the properties of an object in the world.

    Can be robot or YAML-loaded.
    '''

    # Properties that must match for objects to be considered matching
    # (for the purposes of sentence generation).
    matching_properties = ['name', 'color', 'type']

    def __init__(self, properties=None):
        '''
        Use from_dicts if calling from yaml-loaded list of property
        dictionaries.
        '''
        if properties is None:
            properties = {}
        self.properties = properties

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return (
            self.get_property('name') if self.has_property('name')
            else 'unknownObj')

    @staticmethod
    def from_dicts(objs):
        '''
        Args:
            objs ([{str: object}]): List of dicts, each with a mapping
                from property name to value.

                This will likely be the YAML-loaded 'objects' component
                of world dict for basic testing, programmatically-
                constructed for programtic testing, and robot-sensor-
                supplied for real robot usage.

        Returns:
            [WorldObject]
        '''
        return [WorldObject(obj_dict) for obj_dict in objs]

    @staticmethod
    def from_ros(world_objects):
        '''
        Args:
            world_objects (WorldObjects):ROS-msg format WorldObjects.

        Returns:
            [WorldObject]: Array of our own WorldObject format.
        '''
        wobjs = []
        # rwo = ros world object
        for rwo in world_objects.world_objects:
            # Each object here is a WorldObject (as in WorldObject.msg).
            # This was kind of a bad decision in class names, sorry.
            # However, we're not explicitly importing it, so it should
            # be fine.
            #
            # Our task for each object from ROS is to see if a property
            # is really set. If it's not, we just won't include it in
            # our internal representation. The accessors we provide
            # check for property existence, so this is the correct way
            # (rather than copying dummy values).

            # The following are always set (or their default values are
            # such that we can't check).
            obj = {
                'name': rwo.name,
                'is_leftmost': rwo.is_leftmost,
                'is_rightmost': rwo.is_rightmost,
                'is_farthest': rwo.is_farthest,
                'is_nearest': rwo.is_nearest,
                'is_biggest': rwo.is_biggest,
                'is_smallest': rwo.is_smallest,
            }

            # The following may or may not be set.
            if len(rwo.color) > 0:
                obj['color'] = rwo.color
            if len(rwo.type) > 0:
                obj['type'] = rwo.type

            # The following may or may not be set, and they are arrays.
            if len(rwo.is_pickupable) == 2:
                obj['is_pickupable'] = rwo.is_pickupable
            if len(rwo.is_above_reachable) == 2:
                obj['is_above_reachable'] = rwo.is_above_reachable
            if len(rwo.is_nextto_reachable) == 2:
                obj['is_nextto_reachable'] = rwo.is_nextto_reachable

            wobjs += [WorldObject(obj)]
        return wobjs

    def has_property(self, name):
        '''
        Returns whether world object has a property.

        This is really mostly useful for testing, where we might omit
        some world object state.

        Returns:
            bool
        '''
        return name in self.properties

    def get_property(self, name):
        '''
        Gets a world object's property by name.

        Returns:
            object
        '''
        return self.properties[name]

    def matches_for_generation(self, other):
        '''
        Returns whether self matches other enough that sentences do not
        have to be re-generated.

        Args:
            other (WorldObject)

        Returns
            bool
        '''
        for prop in WorldObject.matching_properties:
            if not self.property_match(other, prop):
                return False
        return True

    def property_match(self, other, prop):
        '''
        Returns whether a property matches in two WorldObjects.

        Args:
            other (WorldObject)
            prop (str)

        Returns:
            bool
        '''
        return (
            self.has_property(prop) and
            other.has_property(prop) and
            self.get_property(prop) == other.get_property(prop))


class Robot(object):
    '''
    Provides an interface for accessing robot data.
    '''

    def __init__(self, properties=None):
        '''
        Args:
            properties ({str: object}, optional): Mapping of names to
                properties, which are usually strings, bools, or
                two-element bool lists (for right, left hands). This
                will likely come from a yaml-loaded dictionary in
                basic testing, a programatiicaly-created dictionary in
                programtic testing, and the real robot (via a ROS
                message) in real-robot testing.
        '''
        if properties is None:
            properties = {}
        self.properties = properties

    @staticmethod
    def from_ros(robot_state):
        '''
        Args:
            robot_state (RobotState): From ROS.

        Returns:
            Robot
        '''
        props = {}

        # Uncheckable properties (e.g. bools).
        props['is_executing'] = robot_state.is_executing

        # Check checkable properties before adding.
        # Strings (must have characters).
        if len(robot_state.last_cmd_side) > 0:
            props['last_cmd_side'] = robot_state.last_cmd_side

        # Lists (must be non-empty).
        if len(robot_state.gripper_states) == 2:
            props['gripper_states'] = robot_state.gripper_states

        if len(robot_state.can_move_up) == 2:
            props['can_move_up'] = robot_state.can_move_up

        if len(robot_state.can_move_down) == 2:
            props['can_move_down'] = robot_state.can_move_down

        if len(robot_state.can_move_toleft) == 2:
            props['can_move_toleft'] = robot_state.can_move_toleft

        if len(robot_state.can_move_toright) == 2:
            props['can_move_toright'] = robot_state.can_move_toright

        if len(robot_state.can_move_forward) == 2:
            props['can_move_forward'] = robot_state.can_move_forward

        if len(robot_state.can_move_backward) == 2:
            props['can_move_backward'] = robot_state.can_move_backward

        return Robot(props)

    def has_property(self, name):
        '''
        Returns whether robot has a property.

        This is really mostly useful for testing, where we might omit
        the robot state.

        Returns:
            bool
        '''
        return name in self.properties

    def get_property(self, name):
        '''
        Gets a robot's property by name.

        Returns:
            object
        '''
        return self.properties[name]


class RobotCommand(object):
    '''A Command wrapper that will generate objects in the forms that
    will be returned to the robot (or something close for us humans to
    read).
    '''

    def __init__(self, name, args):
        '''
        Used internally. Use a factory if you're calling this from
        outside this class.

        Args:
            name (str)
            args ([str])
        '''
        self.name = name
        self.args = args

    @staticmethod
    def from_command(command):
        '''
        Factory.

        Args:
            command (Command)

        Returns:
            RobotCommand
        '''
        # Note that we skip the first option in the list, because this
        # is the name of the command itself.
        return RobotCommand(command.get_name(), command.opt_str_list()[1:])

    @staticmethod
    def from_strs(name, args):
        '''
        Factory.

        Args:
            name (str)
            args ([str])

        Returns:
            RobotCommand
        '''
        return RobotCommand(name, args)

    def to_rosmsg(self):
        '''
        Returns ROS representation of this command.
        '''
        from pr2_pbd_interaction.msg import HandsFreeCommand
        return HandsFreeCommand(
            cmd=self.name,
            args=self.args
        )

    def __eq__(self, other):
        '''
        Args:
            other (RobotCommand)

        Returns:
            bool
        '''
        return self.name == other.name and self.args == other.args

    def __ne__(self, other):
        '''
        Args:
            other (RobotCommand)

        Returns:
            bool
        '''
        return self.name != other.name or self.args != other.args

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ': '.join([self.name, ', '.join(self.args)])


class Parser(object):

    # Couple settings (currently for debugging)
    display_limit = 3

    def __init__(self, grammar_yaml=COMMAND_GRAMMAR, buffer_printing=False):
        # If set, logger saves ouput
        Logger.buffer_printing = buffer_printing

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
        self.hfcmd_pub = None
        self.ros_running = False

    def get_print_buffer(self):
        '''
        Returns:
            str
        '''
        return Logger.get_buffer()

    def simple_interactive_loop(self):
        '''
        Answers queries using no world objects and no robot state.
        '''
        self.set_world()
        self.interactive_loop()

    def run_default_query(self):
        '''
        Programmatically runs hardcoded query. Useful for debugging.
        '''
        # Provide initial world, robot
        self.set_default_world()
        utterance = 'move right hand up'
        Info.p(self.parse(utterance)[0])

    def default_interactive_loop(self):
        '''
        Answers queries using file-saved world objects and robot state.
        '''
        # Provide initial world, robot
        self.set_default_world()
        self.interactive_loop()

    def print_sentences(self):
        '''Prints all sentences to stdout, one per line.'''
        Debug.printing = False
        Info.printing = False

        # NOTE(mbforbes): Because sentences can only be generated after
        # we know about the objects that we have in the world, if we
        # want to truly exhaustively generate all possible sentences
        # here then we must exhaustively generate all possible objects
        # first. HOWEVER, with word skipping for objects (you don't
        # always want to have to say all descriptors for an object), the
        # space of possible sentences explodes (into the millions). So
        # we seprately generate all sentences without objects, then
        # separately generate object phrases.
        self.set_world()
        for sentence in self.sentences:
            print sentence.get_raw()

        # Debug
        wobjs = [WorldObject(o) for o in self._gen_objs()]

        # Look for object options
        sentence_set = set()
        self.set_world(world_objects=wobjs)
        for opt in self.options:
            if type(opt) == ObjectOption:
                phrase_lists = opt.get_phrases(True)
                for phrase_list in phrase_lists:
                    sentence_set.add(' '.join([str(p) for p in phrase_list]))
        for sentence in sentence_set:
            print sentence

        # Backup as sentence generation with objects and word skipping
        # explodes.
        for phrase in self.phrases:
            print phrase

    def _gen_objs(self):
        '''For sentence generation (for speech recognition training
        data), must get exhaustive list of objs.
        '''
        all_opts = []
        for k, prop in yaml.load(
                open(COMMAND_GRAMMAR))['descriptors'].iteritems():
            opts = [(k, o) for o in prop['options']]
            all_opts += [opts]

        all_combs = Algo.gen_recursive(all_opts)

        objs = []
        for idx, comb in enumerate(all_combs):
            obj = {'name': 'obj' + str(idx)}
            # Put in this combination of exclusive properties
            for name, val in comb:
                if val in M_WO_INVERSE:
                    # We need to map to some world object key.
                    obj[M_WO_INVERSE[val]] = val
                else:
                    # The key from the grammar was the one we use.
                    obj[name] = val
            objs += [obj]
        return objs

    def startup_ros(self):
        '''ROS-specific: Sets up callbacks for
            - recognized speech from pocketsphinx
            - world state updates
            - robot state updates
        and publishers for
            - HandsFreeCommand
        '''
        # Some settings
        Debug.printing = False

        # Setup default system.
        self.set_world()

        # Setup ROS.
        try:
            import roslib
            roslib.load_manifest('pr2_pbd_interaction')
            import rospy
            from std_msgs.msg import String
            from pr2_pbd_interaction.msg import (
                HandsFreeCommand, WorldObjects, RobotState)
            rospy.init_node('hfpbd_parser')

            # We get: speech, world objects, robot state.
            rospy.Subscriber('recognizer/output', String, self.sphinx_cb)
            rospy.Subscriber(
                'handsfree_worldobjects', WorldObjects, self.world_objects_cb)
            rospy.Subscriber(
                'handsfree_robotstate', RobotState, self.robot_state_cb)

            # We send: parsed commands.
            self.hfcmd_pub = rospy.Publisher(
                'handsfree_command', HandsFreeCommand)

            # Don't die!
            self.ros_running = True
            rospy.spin()
        except ImportError:
            # We don't have ROS installed or running! That's OK.
            pass

    def sphinx_cb(self, recognized):
        '''ROS-specific: Callback for when data received from
        Pocketsphinx.

        Args:
            recognized (String)
        '''
        # Ensure we actually got something.
        recog_str = recognized.data
        if len(recog_str.strip()) == 0:
            return

        # Parse; ROS response happens in parser automatically.
        robotCommand, buf_log = self.parse(recog_str)

    def world_objects_cb(self, world_objects):
        '''ROS-specific: Callback for when world objects are received
        from the robot.

        Args:
            world_objects (WorldObjects)
        '''
        self.update_objects(WorldObject.from_ros(world_objects))

    def robot_state_cb(self, robot_state):
        '''ROS-specific: Callback for when robot state is received from
        the robot.

        Args:
            robot_state (RobotState)
        '''
        self.update_robot(Robot.from_ros(robot_state))

    def interactive_loop(self):
        '''
        Answers queries using the already-set world objects and robot
        state.
        '''
        while True:
            utterance = raw_input('> ')
            Info.p(self.parse(utterance)[0])

    def set_world(self, world_objects=[], robot=Robot()):
        '''
        Updates the objects in the world and the robot.

        Args:
            world_objects ([WorldObject])
            robot ([Robot])
        '''
        self.lock.acquire()
        self.world_objects = world_objects
        self.robot = robot
        self._update_world_internal()
        self.lock.release()

    def update_objects(self, world_objects=[]):
        '''
        Updates only the objects in the world.

        Args:
            world_objects ([WorldObject])
        '''
        self.lock.acquire()
        # The robot must be set to fully update.
        self.world_objects = world_objects
        if self.robot is not None:
            self._update_world_internal()
        self.lock.release()

    def update_robot(self, robot=Robot()):
        '''
        Updates only the robot.

        Args:
            robot ([Robot])
        '''
        self.lock.acquire()
        # The world objects must be set to fully update.
        self.robot = robot
        if self.world_objects is not None:
            self._update_world_internal()
        self.lock.release()

    def parse(self, u):
        '''
        Args:
            u (str): utterance

        Returns:
            (RobotCommand,str): 2-tuple of
                - the top command as a RobotCommand
                - any buffered logging (debug, info, etc.), if logging
                    has been buffered.
        '''
        self.lock.acquire()

        # Sanity check for state.
        if self.world_objects is None or self.robot is None:
            Error.p('Must set Parser world_objects and robot before parse().')
            sys.exit(1)

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
        if (Numbers.are_floats_close(first.lang_score, second.lang_score) and
                Numbers.are_floats_close(first.score, second.score)):
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

        # We return a new representation of the command as well as some
        # logging buffers (perhaps) for display.
        self._log_results(rc)
        ret = (
            rc,
            '\n'.join([self.start_buffer, self.get_print_buffer()])
        )

        self.lock.release()

        # Maybe publish to ROS!
        if self.ros_running and self.hfcmd_pub is not None:
            self.hfcmd_pub.publish(robotCommand.to_rosmsg())

        # Give back to caller
        return ret

    def _log_results(self, rc):
        '''
        Write results of parse to log.

        Args:
            rc (RobotCommand): What we're returning.
        '''
        if Info.printing:
            # Display sentences.
            self.sentences.sort(key=attrgetter('score'), reverse=True)
            Info.p('Top %d sentences:' % (Parser.display_limit))
            for idx, s in enumerate(self.sentences):
                if idx < Parser.display_limit:
                    Info.pl(1, s)

            # Display commands.
            Info.p("Top %d commands:" % (Parser.display_limit))
            for idx, c in enumerate(self.commands):
                if idx < Parser.display_limit:
                    Info.pl(1, c)

            # For clarity, show what we're returning.
            Info.p('Returning command: %s' % (str(rc)))

    def set_default_world(self):
        '''
        Sets "default" (file-specified) world objects and robot.
        '''
        world_dict = yaml.load(open(WORLD))
        w_objects = WorldObject.from_dicts(world_dict['objects'])
        robot = Robot(world_dict['robot'])
        self.set_world(w_objects, robot)

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
        self.start_buffer = self.get_print_buffer()

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
        if Parser._check_objects_match(
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

    @staticmethod
    def _check_objects_match(objs1, objs2):
        '''
        Returns whether two sets of WorldObjects would generate the same
        sentences, and thus new sentence generation is not required.

        Args:
            objs1 ([WorldObject]|None): List of WorldObjects. Can be
                None or empty.
            objs2 ([WorldObject]|None): List of WorldObjects. Can be
                None or empty.

        Returns:
            bool
        '''
        if objs1 is None and objs2 is None:
            return True

        if ((objs1 is None and objs2 is not None) or
                (objs1 is not None and objs2 is None)):
            Debug.p('Objects mismatch: only one list is not none.')
            return False

        if len(objs1) != len(objs2):
            Debug.p('Objects mismatch: lengths do not match.')
            return False

        if len(objs1) == 0 and len(objs2) == 0:
            return True

        # If we've come this far, there are two non-empty object lists,
        # and we need to see whether each contains one in the other.
        # Checking one way is not sufficient in general, in case one has
        # duplicate types of objects and the other does not.
        return (
            Parser._check_objects_found_in(objs1, objs2) and
            Parser._check_objects_found_in(objs2, objs1))

    @staticmethod
    def _check_objects_found_in(objs1, objs2):
        '''
        Returns whether each of objs1 is found in objs2.

        Args:
            objs1 ([WorldObject]): Non-empty list of WorldObjects.
            objs2 ([WorldObject]): Non-empty list of WorldObjects.

        Returns:
            bool
        '''
        # Debug.p('Checking ' + str(objs1) + ' vs ' + str(objs2))
        for obj1 in objs1:
            match_found = False
            for obj2 in objs2:
                if obj1.matches_for_generation(obj2):
                    match_found = True
                    break
            if not match_found:
                # Debug.p('Could not find ' + str(obj1) + ' in ' + str(objs2))
                return False
        return True

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
        Numbers.normalize(self.commands, min_score=MIN_SCORE)

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


########################################################################
# Main and experimental stuff.
########################################################################

def play(parser):
    '''
    Where I try CRAZY stuff out.

    ...like whether things are broken.

    Args:
        parser (Parser): Just initialized to the grammar, un-query'd.
    '''
    o_full_reachable = {
        'name': 'obj0',
        # Relation to arms.
        'is_pickupable': [True, True],
        'is_above_reachable': [True, True],
        'is_nextto_reachable': [True, True],
        'is_leftof_reachable': [True, True],
        'is_rightof_reachable': [True, True],
        'is_frontof_reachable': [True, True],
        'is_behind_reachable': [True, True],
        'is_topof_reachable': [True, True],
        'is_near_reachable': [True, True],
        # Relation to other objects. These should be more general.
        'is_leftmost': False,
        'is_rightmost': False,
        'is_biggest': False,
        'is_smallest': False,
        # E.g. red, blue, green, unknown
        'color': 'red',
        # E.g. cup, box, unknown
        'type': 'box',
    }
    o_full_reachable_second = {
        'name': 'obj1',
        # Relation to arms.
        'is_pickupable': [True, True],
        'is_above_reachable': [True, True],
        'is_nextto_reachable': [True, True],
        'is_leftof_reachable': [True, True],
        'is_rightof_reachable': [True, True],
        'is_frontof_reachable': [True, True],
        'is_behind_reachable': [True, True],
        'is_topof_reachable': [True, True],
        'is_near_reachable': [True, True],
        # Relation to other objects. These should be more general.
        'is_leftmost': False,
        'is_rightmost': False,
        'is_biggest': False,
        'is_smallest': False,
        # E.g. red, blue, green, unknown
        'color': 'blue',
        # E.g. cup, box, unknown
        'type': 'cup',
    }
    o_right_possible = {
        'name': 'obj0',
        # Relation to arms.
        'is_pickupable': [True, False],
        'is_above_reachable': [True, False],
        'is_nextto_reachable': [True, False],
        'is_leftmost': False,
        'is_rightmost': True,
        'is_biggest': False,
        'is_smallest': True,
        # E.g. red, blue, green, unknown
        'color': 'red',
        # E.g. cup, box, unknown
        'type': 'box',
    }
    o_left_possible_second = {
        'name': 'obj1',
        # Relation to arms.
        'is_pickupable': [False, True],
        'is_above_reachable': [False, True],
        'is_nextto_reachable': [False, True],
        'is_leftmost': True,
        'is_rightmost': False,
        'is_biggest': True,
        'is_smallest': False,
        # E.g. red, blue, green, unknown
        'color': 'red',
        # E.g. cup, box, unknown
        'type': 'box',
    }
    objs = [
        WorldObject(o_full_reachable),
        WorldObject(o_full_reachable_second),
        # WorldObject(o_right_possible),
        # WorldObject(o_left_possible_second),
    ]

    parser.set_world(world_objects=objs)
    # parser.print_sentences()
    # parser.update_robot(robot=Robot({'last_cmd_side': 'left_hand'}))
    # parser.set_world(world_objects=objs)
    parser.interactive_loop()


def main():
    arg = None
    if len(sys.argv) > 1:
        arg = sys.argv[1]

    parser = Parser()
    if arg is None:
        parser.run_default_query()
    elif arg == 'interactive':
        parser.default_interactive_loop()
    elif arg == 'interactive-simple':
        parser.simple_interactive_loop()
    elif arg == 'sentences':
        parser.print_sentences()
    elif arg == 'ros':
        parser.startup_ros()
    elif arg == 'play':
        play(parser)
    else:
        Error.p("Unknown option: " + arg)

if __name__ == '__main__':
    main()
