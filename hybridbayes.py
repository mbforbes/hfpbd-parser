'''Hybrid approach: Bayes conceptually, score functions in low-level.

Here's the process:
    - Extract all Options, which contain Phrases
        - WordOptions' phrases come from a template (COMMAND_GRAMMAR)
        - ObjectOptions' phrases come from the object properties
            (objects are loaded into WorldObjects)

    - Use the Options to build Parameters

    - Use the Paramters to build CommandTemplates

    - Use the CommandTemplates to generate all Commands

    - Apply the world (W) and robot (R) states into the Commands to get
        a prior over which are most likely (P(C|W,R)). (Command.score)

    - Use all Commands to generate all Sentences

    - Pre-match-score all Commands with Sentences based on Phrase
        matches (P(C|L)).

    - When an utterance comes in, score all Sentences based on Phrase
        matching strategies (subclass of MatchingStrategy) (P(L))

    - Combine Command-Sentences scores with the Sentence-utterance
        scores to find the language score. (Command.lang_score)

    - Combine the W, R-induced probability P(C|W,R) with the language
        score P(L|W,R,C) to get the final probability P(C|W,R,C,L)
        (Command.final_p).
'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import code
from collections import OrderedDict
import copy
import pprint
import sys
import threading
import yaml

# Local
from util import Logger, Error, Info, Debug, Numbers
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
}

# Map (M) from component options to robot properties (RP).
M_RP = {
    'up': 'can_move_up',
    'down': 'can_move_down',
    'to_left': 'can_move_toleft',
    'to_right': 'can_move_toright',
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

# General scoring
LENGTH_EXP = 10.0  # Polynomial degree for weight by length (x^this).

# Command score
START_SCORE = 1.0  # To begin with. Maybe doesn't matter.
MIN_SCORE = 0.1  # What to boost scores to *before* normalizing.
P_LOCUNR = -5.0  # Penalty: the requested location is unreachable.
P_OBJUNR = -5.0  # Penalty: the requested object cannot be picked up.
P_NOTLASTSIDE = -0.1  # Penalty: the side wasn't the last commanded.
P_GSTATE = -8.0  # Penalty: nonsensical gripper state change requested.
P_BADPP = -5.0  # Penalty: bad pickup/place command requested (from GS)

# Language score

# Combined Score
LANG_WEIGHT = 2.0  # How much to weight rel. to cmd (cmd wegiht = 1.0)


########################################################################
# Classes
########################################################################

class CommandDict:
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

    def make_templates(self, wobjs):
        '''
        Args:
            wobjs ([WorldObject]): The object we see in the world (or
                from a YAML file).

        Returns:
            [CommandTemplate]
        '''
        options = self._make_options(wobjs)
        parameters = self._make_parameters(options)
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

    def _make_options(self, wobjs):
        '''
        Args:
            wobjs ([WorldObject]): The object wes see in the world (or
                from a YAML file or python dict).

        Returns:
            {str: Option}: Map of option name: Option.
        '''
        options = {}
        # Make word options from phrases.
        matcher = DefaultMatcher
        for opt_name, props in self.ydict['options'].iteritems():
            # Get strategy, if special
            if 'strategy' in props:
                matcher_name = props['strategy']
                matcher = Matchers.MATCHERS[matcher_name]
            else:
                matcher = Matchers.MATCHERS['default']

            # Get phrases
            raw_phrases = props['phrases']
            phrases = [Phrase(phrase, matcher) for phrase in raw_phrases]

            # Make our word option and add it.
            w_opt = WordOption(opt_name, phrases)
            options[opt_name] = w_opt

        # Now make object options, because they are described by word
        # options.
        for wobj in wobjs:
            options[wobj.get_property('name')] = ObjectOption(wobj, options)

        Debug.p('Options: ' + str(options.values()))
        return options


class CommandTemplate:
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


class Command:
    '''A fully-instantiated command.

    Has state: YES
    '''

    # Whether to do some internal checking during each parse.
    debug = False

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

        # P(C|W,R)
        self.score = START_SCORE

        # P(L|C)
        self.sentence_match_probs = {}

        # P(L|C) * P(L) (marginalize across L)
        self.lang_score = 0.0

        # P(C|W,R) * P(L|W,R,C)
        self.final_p = 0.0

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
        if self.lang_score != 0.0:
            score_str += ", lang_score: %0.4f" % (self.lang_score)
        if self.final_p != 0.0:
            score_str += ", final_p: %0.4f" % (self.final_p)
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

    @staticmethod
    def apply_l_precheck(commands, sentences):
        '''
        Checks that numbers are sane. Should be called before apply_l
        (not enforced).

        This does nothing if Command.debug is False.

        Args:
            commands ([Command])
            sentences ([Sentence])
        '''
        if not Command.debug:
            return

        # Ensure command scores are normalized.
        sum_ = sum(c.score for c in commands)
        if not Numbers.are_floats_close(sum_, 1.0):
            Error.p('Command scores are not normed: %0.3f' % (sum_))
            sys.exit(1)

        # Ensure sentence scores normalized.
        # sum_ = sum(s.score for s in sentences)
        # if not Numbers.are_floats_close(sum_, 1.0):
        #     Error.p('Sentence scores are not normed: %0.3f' % (sum_))
        #     sys.exit(1)

        # Check a few things for all commands.
        for c in commands:
            # Quick check whether sentence_match_probs reasonable.
            if len(c.sentence_match_probs) != len(sentences):
                Error.p('Must score_match_sentences before apply_l.')
                sys.exit(1)

            # Ensure all sentences pre-matched with this command.
            for s in sentences:
                if s not in c.sentence_match_probs.keys():
                    Error.p(
                        ('Command %s sentence_match_probs missing ' +
                            'sentence: %s') % (str(c), str(s)))
                    sys.exit(1)

            # # Ensure sentence match scores normalized.
            # sum_ = sum(
            #     p for s, p in c.sentence_match_probs.iteritems())
            # if not Numbers.are_floats_close(sum_, 1.0):
            #     Error.p(
            #         'Command sentence_match_probs not normed: %0.3f'
            #         % (sum_))
            #     sys.exit(1)

    def generate_sentences(self):
        '''
        Returns:
            [Sentence]
        '''
        phrase_lists = Numbers.gen_phrases(self.option_map.values())
        return [Sentence(pl) for pl in phrase_lists]

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
                that options' commands.
        '''
        phrase_sets = []
        for pname, opt in self.option_map.iteritems():
            opt_phrase_sets = opt.get_phrases()
            for phrase in opt_phrase_sets:
                phrase_sets += [phrase]
        return phrase_sets

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
        # All robot applications so far involve a side.
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
            if self.name == 'close':
                if gs == 'closed_empty' or gs == 'has_obj':
                    self.score += P_GSTATE
            if self.name == 'pick_up':
                if gs == 'has_obj':
                    self.score += P_BADPP
            if self.name == 'place':
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
        # Display settings.
        display_limit = 8

        # Debug.p('Sentence-matching command: ' + str(self))
        # Empty list before we start adding to it.
        if len(self.sentence_match_probs) != 0:
            self.sentence_match_probs = {}

        scores = []
        # kvs = []  # Debug
        phrase_sets = self.get_phrase_sets()
        for sentence in sentences:
            score = 0.0
            s_phrases = sentence.get_phrases()

            # We simply count the phrase matches. We want to penalize
            # missing phrases in either direction, so we just check both
            # ways (double-scoring matches). Also note we match on the
            # command side per option (set of phrases).

            # Check whether any phrase for each command option in
            # sentence phrases.
            n_opts = len(phrase_sets)
            match_score = 0.0
            for idx, phrase_set in enumerate(phrase_sets):
                for phrase in phrase_set:
                    # The following uses Phrase's __eq__ method.
                    if phrase in s_phrases:
                        # Verbs get a different score than params.
                        if idx == 0:
                            match_score += MatchingStrategy.verb_match
                        else:
                            match_score += MatchingStrategy.param_match
                        break
            max_match_score = (
                MatchingStrategy.verb_match +
                (n_opts - 1) * MatchingStrategy.param_match)
            score += (match_score / max_match_score)**LENGTH_EXP

            # Check each phrase in the sentence's phrases.
            n_phrases = len(s_phrases)
            match_score = 0.0
            for idx, s_phrase in enumerate(s_phrases):
                for phrase_set in phrase_sets:
                    # The following uses Phrase's __eq__ method.
                    if s_phrase in phrase_set:
                        # Verbs get a different score than params.
                        if idx == 0:
                            match_score += MatchingStrategy.verb_match
                        else:
                            match_score += MatchingStrategy.param_match
                        break
            max_match_score = (
                MatchingStrategy.verb_match +
                (n_phrases - 1) * MatchingStrategy.param_match)
            score += (match_score / max_match_score)**LENGTH_EXP

            scores += [score]
            # Debug.pl(1, '%0.10f %s' % (score, str(sentence.get_raw())))

        # Normalize, save
        norm_scores = Numbers.normalize_list(scores, 0.1, 2.0)
        for idx, sentence in enumerate(sentences):
            self.sentence_match_probs[sentence] = norm_scores[idx]
            # kvs += [(norm_scores[idx], scores[idx], sentence.get_raw())]

        # Debug
        # Debug.p(
        #     'Sentence-matching scores for command: ' + str(self.pure_str()))
        # skvs = sorted(kvs, key=lambda x: -x[0])
        # for kv in skvs:
        #     Debug.pl(1, "%0.3f %0.3f %s" % (kv[0], kv[1], kv[2]))

    def apply_l(self, sentences):
        '''
        Applies utterance-scored sentences to command-matched score.

        Must first call score_match_sentences(...).

        Args:
            sentences ([Sentence])
        '''
        score_total = 0.0
        # Debug.pl(0, 'Scoring %s' % (str(self)))
        for s in sentences:
            inc = s.score * self.sentence_match_probs[s]
            score_total += inc
            # Debug.pl(
            #     1,
            #     'Tot: %0.3f, adding %0.3f = %0.3f x %0.3f from %s' % (
            #         score_total,
            #         inc,
            #         self.sentence_match_probs[s],
            #         s.score,
            #         s.get_raw()
            #     )
            # )
        # Debug.pl(1, 'Total: %0.3f' % (score_total))
        self.lang_score = score_total

    def get_final_p(self):
        '''
        Calculates final probability of command by combining P(C|W,R)
        and P(L|W,R).

        The result is stored in self.final_p.
        '''
        # self.final_p = self.score * self.lang_score  # original
        self.final_p = self.score + self.lang_score * LANG_WEIGHT  # new


class Parameter:
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


class Option:
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


class WordOption(Option):
    '''Holds options info, including all possible referring phrases.'''

    def __init__(self, name, phrases):
        self.name = name
        self.phrases = phrases

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
        # First, we do the unique properties (left-most, biggest, etc.)
        word_options = []
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

    def get_phrases(self):
        '''
        Returns a list of possible phrases, where each element is a list
        of Phrases.

        Returns:
            [[Phrase]]
        '''
        return Numbers.gen_phrases(self.word_options)
        # return [Phrase(self.name, DefaultMatcher)]  # just use 'objn'


class Sentence:
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

    def score_match(self, utterance):
        '''
        Args:
            utterance (str)
        '''
        total_score = 0.0
        # Debug.pl(1, "Scoring sentence: " + str(self))
        for phrase in self.phrases:
            phrase_score = phrase.score(utterance)
            total_score += phrase_score
            # Debug
            # Debug.pl(
            #     2,
            #     "%0.2f  score of phrase: %s" % (phrase_score, str(phrase)))

        # Option 1: Weight the scores by the max (i.e. the length).
        max_score = (
            MatchingStrategy.verb_match +
            (len(self.phrases) - 1) * MatchingStrategy.param_match)
        self.score = (total_score / max_score)**LENGTH_EXP
        # Debug.pl(2, "Final score: " + str(self))

        # Option 2: Just use the raw score?
        # self.score = total_score


class Phrase:
    '''Holds a set of words and a matching strategy for determining if
    it is matched in an utterance.'''

    def __init__(self, words, strategy):
        '''
        Args:
            words ([str])
            strategy (MatchingStrategy)
        '''
        self.words = words
        self.strategy = strategy

    def score(self, utterance):
        '''
        Args:
            utterance (str)

        Returns:
            float
        '''
        return self.strategy.match(self.words, utterance)

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return self.words

    def __eq__(self, other):
        '''
        Args:
            other (Phrase)

        Returns:
            bool
        '''
        return self.words == other.words

    def __ne__(self, other):
        '''
        Args:
            other (Phrase)

        Returns:
            bool
        '''
        return self.words != other.words


class WorldObject:
    '''
    Contains data about the properties of an object in the world.

    Can be robot or YAML-loaded.
    '''

    def __init__(self, properties=None):
        '''
        Use from_dicts if calling from yaml-loaded list of property
        dictionaries.
        '''
        if properties is None:
            properties = {}
        self.properties = properties

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


class Robot:
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


class RobotCommand:
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


class Parser:

    # Couple settings (currently for debugging)
    display_limit = 25

    def __init__(
            self, grammar_yaml=COMMAND_GRAMMAR, buffer_printing=False,
            debug=False):
        # If set, logger saves ouput
        Logger.buffer_printing = buffer_printing
        # If set, do some internal checking during each parse.
        Command.debug = debug

        # Load
        self.command_dict = CommandDict(yaml.load(open(grammar_yaml)))

        # We can't be updating our guts while we try to churn something
        # out.
        self.lock = threading.Lock()

        # Initialize (for clarity)
        self.world_objects = None
        self.robot = None
        self.templates = None
        self.commands = None

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
        # Provide initial world, robot
        Debug.printing = False
        Info.printing = False
        # NOTE(mbforbes): Because sentences can only be generated after
        # we know about the objects that we have in the world, if we
        # want to truly exhaustively generate all possible sentences
        # here then we must exhaustively generate all possible objects
        # first. Should there be a helper function that does that?
        self.set_default_world()
        for sentence in self.sentences:
            print sentence.get_raw()

    def startup_ros(self):
        '''ROS-specific: Sets up callbacks for
            - recognized speech from pocketsphinx
            - world state updates (TODO)
            - robot state updates (TODO)
        and publishers for
            - HandsFreeCommand
        '''
        # Setup default system.
        self.set_world()

        # Setup ROS.
        import roslib
        roslib.load_manifest('pr2_pbd_interaction')
        import rospy
        from std_msgs.msg import String
        from pr2_pbd_interaction.msg import HandsFreeCommand
        rospy.init_node('hfpbd_parser')
        rospy.Subscriber('recognizer/output', String, self.sphinx_cb)
        self.hfcmd_pub = rospy.Publisher('handsfree_command', HandsFreeCommand)
        rospy.spin()

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

        # Parse and respond!
        robotCommand, buf_log = self.parse(recog_str)
        self.hfcmd_pub.publish(robotCommand.to_rosmsg())

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

    def parse(self, utterance):
        '''
        Args:
            utterance (str)

        Returns:
            (RobotCommand,str): 2-tuple of
                - the top command as a RobotCommand
                - any buffered logging (debug, info, etc.), if logging
                    has been buffered.
        '''
        self.lock.acquire()
        if self.world_objects is None or self.robot is None:
            Error.p('Must set Parser world_objects and robot before parse().')
            sys.exit(1)

        Info.p("Parser received utterance: " + utterance)

        # Score sentences.
        for s in self.sentences:
            s.score_match(utterance)
        # Numbers.boost(self.sentences)  # boost? or
        Numbers.normalize(self.sentences, 'score')  # norm?
        self.sentences = sorted(self.sentences, key=lambda x: -x.score)

        # Apply L.
        Command.apply_l_precheck(self.commands, self.sentences)
        for c in self.commands:
            c.apply_l(self.sentences)
        # Numbers.boost(self.commands, 'lang_score')  # boost? or
        Numbers.normalize(self.commands, 'lang_score')  # norm?

        # Get combined probability.
        for c in self.commands:
            c.get_final_p()
        Numbers.normalize(self.commands, 'final_p')  # norm?

        # Display sentences.
        Info.p('Top %d sentences:' % (Parser.display_limit))
        for idx, s in enumerate(self.sentences):
            if idx < Parser.display_limit:
                Info.pl(1, s)

        # Display commands.
        self.commands = sorted(self.commands, key=lambda x: -x.final_p)
        ptot = 0.0
        Info.p("Top %d commands:" % (Parser.display_limit))
        for idx, c in enumerate(self.commands):
            if idx < Parser.display_limit:
                Info.pl(1, c)
            ptot += c.final_p
        Info.p('Total (%d): %0.2f' % (len(self.commands), ptot))

        ret = (
            RobotCommand.from_command(self.commands[0]),
            '\n'.join([self.start_buffer, self.get_print_buffer()])
        )
        self.lock.release()

        return ret

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
        # Make templates (this extracts options and params).
        self.templates = self.command_dict.make_templates(self.world_objects)
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

        # Apply W and R to weight C prior.
        for c in self.commands:
            c.apply_w()
            c.apply_r(self.robot)
        Numbers.normalize(self.commands, min_score=MIN_SCORE)

        # Pre-score commands with all possible sentences.
        for c in self.commands:
            c.score_match_sentences(self.sentences)  # Auto-normalizes.

        # Display commands.
        self.commands = sorted(self.commands, key=lambda x: -x.score)
        ptot = 0.0
        Info.p("Top commands (before utterance):")
        for idx, c in enumerate(self.commands):
            if idx < Parser.display_limit:
                Info.pl(1, c)
            ptot += c.score
        Info.p('Total (%d): %0.2f' % (len(self.commands), ptot))

        self.start_buffer = self.get_print_buffer()


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
    o_right_possible = {
        'name': 'obj0',
        # Relation to arms.
        'is_pickupable': [True, False],
        'is_above_reachable': [True, False],
        'is_nextto_reachable': [True, False],
        'is_leftmost': False,
        'is_righttmost': True,
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
        'is_righttmost': False,
        'is_biggest': True,
        'is_smallest': False,
        # E.g. red, blue, green, unknown
        'color': 'red',
        # E.g. cup, box, unknown
        'type': 'box',
    }
    objs = [
        WorldObject(o_right_possible),
        WorldObject(o_left_possible_second),
    ]

    parser.set_world(world_objects=objs)
    # parser.print_sentences()
    # parser.update_robot(robot=Robot({'last_cmd_side': 'left_hand'}))
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
