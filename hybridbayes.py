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
        a prior over which are most likely (P(C|W,R)).

    - Use all Commands to generate all Sentences

    - Pre-match-score all Commands with Sentences based on Phrase
        matches (P(C|L))

    - When an utterance comes in, score all Sentences based on Phrase
        matching strategies (subclass of MatchingStrategy) (P(L))

    - Weight Command-Sentences scores with the Sentence-utterance scores
        to find the most likely command C.
'''

########################################################################
# Imports
########################################################################

# Builtins
import code
from collections import OrderedDict
import copy
import pprint
import sys
import yaml

########################################################################
# Module-level constants
########################################################################

# Global options
DEBUG = True  # Turn off for significant speedups.

ERROR_PRINTING_DEFAULT = True
INFO_PRINTING_DEFAULT = True
DEBUG_PRINTING_DEFAULT = True

COMMAND_GRAMMAR = 'commands.yml'
WORLD = 'world.yml'
OBJ_PARAM = 'obj'

FLOAT_COMPARE_EPSILON = 0.001

# How to index into object properties by side
S = {'right': 0, 'left': 1}

# Map (M) from component options to object properties (OP).
M_OP = {
    'above': 'is_above_reachable',
    'nextto': 'is_nextto_reachable',
}

# Map (M) from component options to robot properties (RP).
M_RP = {
    'up': 'can_move_up',
    'down': 'can_move_down',
    'toleft': 'can_move_toleft',
    'toright': 'can_move_toright',
}

# Command score
START_SCORE = 0.0  # To begin with. Maybe doesn't matter.
MIN_SCORE = 10.0  # Minimum score for normalizing. Only adds to match.
P_LOCUNR = -50  # Penalty: the requested location is unreachable.
P_OBJUNR = -50  # Penalty: the requested object cannot be picked up.
P_NOTLASTSIDE = -10  # Penalty: the side wasn't the last commanded.
P_GSTATE = -80  # Penalty: nonsensical gripper state change requested.
P_BADPP = -50  # Penalty: bad pickup/place command requested (from GS)

# Language score
LANG_MATCH_SCORE = 90
LANG_UNMATCH_SCORE = -90
CMD_PHRASE_MATCH_SCORE = 90
CMD_PHRASE_UNMATCH_SCORE = -90


########################################################################
# Classes
########################################################################

class Logger:
    printing = False
    prefix = '[IMPLEMENT ME]'

    @classmethod
    def p(cls, obj):
        cls.pl(0, obj)

    @classmethod
    def pl(cls, level, obj):
        '''
        Args:
            level (int): how many tabs (one space if 0)
            obj (Object): what to print
        '''
        string = str(obj)
        if cls.printing:
            indent = ' ' if level == 0 else '\t' * (level + 1)
            print ''.join([cls.prefix, indent, string])


# Error
class Error(Logger):
    printing = ERROR_PRINTING_DEFAULT
    prefix = '[ERROR]'


# Debugging
class Info(Logger):
    printing = INFO_PRINTING_DEFAULT
    prefix = '[INFO]'


# Debugging
class Debug(Logger):
    printing = DEBUG_PRINTING_DEFAULT
    prefix = '[DEBUG]'


class Util:
    '''
    Misc. helper functionality.
    '''

    @staticmethod
    def are_floats_close(a, b, epsilon=FLOAT_COMPARE_EPSILON):
        '''Checks whether two floats are within epsilon of each other.

        Args:
            a (float): One number.
            b (float): The other number.
            epsilon (float): Acceptable wiggle room (+/-) between a and
                b.

        Returns:
            bool: Whether a and b are within epsilon of each other.
        '''
        # We try to do this in an overflow-friendly way, though it
        # probably isn't a big deal with our use cases and python.
        return a - epsilon <= b if a > b else b - epsilon <= a

    @staticmethod
    def normalize_list(nums, min_score=MIN_SCORE):
        '''
        Normalizes list of floats to a valid probability distribution.

        Args:
            nums ([float]):
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to MIN_SCORE.

        Returns:
            [float]
        '''
        # First, boost all to some minimum value.
        min_ = min(nums)
        boost = MIN_SCORE - min_ if MIN_SCORE > min_ else 0.0
        nums = [n + boost for n in nums]

        # Next, do the normalization.
        sum_ = sum(nums)
        return [n / sum_ for n in nums]

    @staticmethod
    def normalize(objs, attr='score', min_score=MIN_SCORE):
        '''
        Normalizes list of objects with a attr attribute (that is a
        float) to a valid probability distribution.

        Args:
            objs ([Object]): List of Objects
            attr (str, optional): The name of the attribute to extract
                from objects. Defaults to 'score'.
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to MIN_SCORE.
        '''
        nums = [getattr(obj, attr) for obj in objs]
        nums = Util.normalize_list(nums, min_score)
        for i in range(len(objs)):
            setattr(objs[i], attr, nums[i])


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
            wobjs ([WorldObject]): The object we see in the world (or
                from a YAML file).

        Returns:
            {str: Option}
        '''
        # Make object options first.
        options = {}
        for wobj in wobjs:
            options[wobj.properties['name']] = ObjectOption(wobj)

        # Go bottom-up.
        matcher = DefaultMatcher
        for opt_name, props in self.ydict['options'].iteritems():
            # Get strategy, if special
            if 'strategy' in props:
                matcher_name = props['strategy']
                matcher = Matchers.MATCHERS[matcher_name]

            # Get phrases
            raw_phrases = props['phrases']
            phrases = [Phrase(phrase, matcher) for phrase in raw_phrases]

            # Make our word option and add it.
            w_opt = WordOption(opt_name, phrases)
            options[opt_name] = w_opt

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

    Here's the process:

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
        lnl = CommandDict.longest_cmd_name_len
        score_str = "score: %0.7f" % (self.score)
        if self.lang_score != 0.0:
            score_str += ", lang_score: %0.7f" % (self.lang_score)
        if self.final_p != 0.0:
            score_str += ", final_p: %0.7f" % (self.final_p)
        padding = 0 if len(self.name) == lnl else lnl - len(self.name)
        name_str = self.name + ' ' * padding
        opt_str = ', '.join(
            [': '.join(
                [str(k), str(v)]) for k, v in self.option_map.iteritems()])
        return "%s   %s   %s" % (score_str, name_str, opt_str)

    @staticmethod
    def apply_l_precheck(commands, sentences):
        '''
        Checks that numbers are sane. Should be called before apply_l
        (not enforced).

        This does nothing if DEBUG is False.

        Args:
            commands ([Command])
            sentences ([Sentence])
        '''
        if not DEBUG:
            return

        # Ensure command scores are normalized.
        sum_ = sum(c.score for c in commands)
        if not Util.are_floats_close(sum_, 1.0):
            Error.p('Command scores are not normed: %0.3f' % (sum_))
            sys.exit(1)

        # Ensure sentence scores normalized.
        sum_ = sum(s.score for s in sentences)
        if not Util.are_floats_close(sum_, 1.0):
            Error.p('Sentence scores are not normed: %0.3f' % (sum_))
            sys.exit(1)

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

            # Ensure sentence match scores normalized.
            sum_ = sum(p for s, p in c.sentence_match_probs.iteritems())
            if not Util.are_floats_close(sum_, 1.0):
                Error.p(
                    'Command sentence_match_probs not normed: %0.3f' % (sum_))
                sys.exit(1)

    @staticmethod
    def _gen_phrases(todo, results=[]):
        '''
        Args:
            todo [Option]
            results ([[Phrase]])

        Returns:
            [[Phrase]]
        '''
        if len(todo) == 0:
            return results
        opt = todo.pop(0)
        next_phrases = opt.get_phrases()
        if results == []:
            new_results = [[p] for p in next_phrases]
        else:
            new_results = []
            for phrase in next_phrases:
                for r in results:
                    new_results += [r + [phrase]]
        return Command._gen_phrases(todo, new_results)

    def generate_sentences(self):
        '''
        Returns:
            [Sentence]
        '''
        opt_copy = self.option_map.values()[:]  # Just copy all refs.
        phrase_lists = Command._gen_phrases(opt_copy)
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
        return [
            opt.get_phrases() for pname, opt in self.option_map.iteritems()]

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
            relpos = self.option_map['relpos'].name
            side = self.option_map['side'].name
            reachables = wobj.properties[M_OP[relpos]]
            if side in ['right', 'left']:
                loc_reachable = reachables[S[side]]
            else:
                # Side == both
                loc_reachable = reachables[0] and reachables[1]
            if not loc_reachable:
                self.score += P_LOCUNR

        # cmds: pickup (params: side)
        if self.name in ['pickup']:
            side = self.option_map['side'].name
            reachables = wobj.properties['is_pickupable']
            if side in ['right', 'left']:
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
            robot (dict)
        '''
        cmd_side = (
            self.option_map['side'].name
            if 'side' in self.option_map else None)
        side_idx = S[cmd_side] if cmd_side is not None else None

        # Side: last moved? Less likely down.
        if self.template.has_params(['side']):
            last_cmd_side = robot['last_cmd_side']
            if (last_cmd_side != 'neither' and
                    self.option_map['side'].name != last_cmd_side):
                self.score += P_NOTLASTSIDE

        # Open/closed & pickup/place: Basd on state (impossible down).
        if self.name == 'open':
            gs = robot['gripper_states'][side_idx]
            if gs == 'open':
                c.score += P_GSTATE
        if self.name == 'close':
            gs = robot['gripper_states'][side_idx]
            if gs == 'closed_empty' or gs == 'has_obj':
                self.score += P_GSTATE
        if self.name == 'pickup':
            gs = robot['gripper_states'][side_idx]
            if gs == 'has_obj':
                self.score += P_BADPP
        if self.name == 'place':
            gs = robot['gripper_states'][side_idx]
            if gs == 'open' or gs == 'closed_empty':
                self.score += P_BADPP

        # move_abs: absdir possible to move to
        if self.name == 'move_abs':
            robot_prop = M_RP[self.option_map['absdir'].name]
            loc_reachable = robot[robot_prop][side_idx]
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
        Debug.p('Sentence-matching command: ' + str(self))
        # Empty list before we start adding to it.
        if len(self.sentence_match_probs) != 0:
            self.sentence_match_probs = {}

        scores = []
        phrase_sets = self.get_phrase_sets()
        for sentence in sentences:
            score = START_SCORE
            s_phrases = sentence.get_phrases()

            # We simply count the phrase matches. We want to penalize
            # missing phrases in either direction, so we just check both
            # ways (double-scoring matches). Also note we match on the
            # command side per option (set of phrases).

            # Check whether any phrase for each command option in
            # sentence phrases.
            for phrase_set in phrase_sets:
                phrase_match = False
                for phrase in phrase_set:
                    if phrase in s_phrases:
                        phrase_match = True
                if phrase_match:
                    score += CMD_PHRASE_MATCH_SCORE
                else:
                    score += CMD_PHRASE_UNMATCH_SCORE

            # Check each phrase in the sentence's phrases.
            for s_phrase in s_phrases:
                phrase_match = False
                for phrase_set in phrase_sets:
                    if s_phrase in phrase_set:
                        phrase_match = True
                if phrase_match:
                    score += CMD_PHRASE_MATCH_SCORE
                else:
                    score += CMD_PHRASE_UNMATCH_SCORE

            scores += [score]
            Debug.pl(1, '%0.2f %s' % (score, str(sentence)))

        # Normalize, save
        scores = Util.normalize_list(scores)
        for idx, sentence in enumerate(sentences):
            self.sentence_match_probs[sentence] = scores[idx]

    def apply_l(self, sentences):
        '''
        Applies utterance-scored sentences to command-matched score.

        Must first call score_match_sentences(...).

        Args:
            sentences ([Sentence])
        '''
        score_total = 0.0
        for s in sentences:
            score_total += s.score * self.sentence_match_probs[s]
        self.lang_score = score_total

    def get_final_p(self):
        '''
        Calculates final probability of command by combining P(C|W,R)
        and P(L|W,R).

        The result is stored in self.final_p.
        '''
        self.final_p = self.score * self.lang_score


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

    def get_phrases(self):
        Error.p("Option:get_phrases unimplemented as it's an interface.")
        sys.exit(1)

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ''.join(['|', self.name, '|'])


class WordOption(Option):
    '''Holds options info, including all possible referring phrases.'''

    def __init__(self, name, phrases):
        self.name = name
        self.phrases = phrases

    def get_phrases(self):
        return self.phrases


class ObjectOption(Option):
    '''Holds options info for object, so can generate all possible
    referring phrases.'''

    def __init__(self, world_obj):
        '''
        Args:
            world_obj (WorldObject)
        '''
        # self.param_name = OBJ_PARAM
        self.name = world_obj.properties['name']
        self.world_obj = world_obj

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
        Returns:
            [Phrase]
        '''
        # TODO(mbforbes): Implement this fully using object properties.
        return [Phrase(self.name, DefaultMatcher)]


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
        return "%0.4f  %s" % (self.score, phrases)

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
        # Debug.pl(1, "Scoring sentence: " + str(self))
        for phrase in self.phrases:
            phrase_score = phrase.score(utterance)
            # Debug.pl(
            #     2,
            #     "%0.2f  score of phrase: %s" % (phrase_score, str(phrase)))
            self.score += phrase_score


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
        if self.strategy.match(self.words, utterance):
            return LANG_MATCH_SCORE
        else:
            return LANG_UNMATCH_SCORE

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return self.words

    def __eq__(self, other):
        '''
        Returns:
            bool
        '''
        return self.words == other.words

    def __ne__(self, other):
        '''
        Returns:
            bool
        '''
        return self.words != other.words


class MatchingStrategy:
    '''Interface for matching strategies.'''

    @staticmethod
    def match(words, utterance):
        Error.p("MatchingStrategy:match unimplemented as it's an interface.")
        sys.exit(1)


class DefaultMatcher:
    '''Default matching strategy.'''

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        return words in utterance


class NotSideMatcher:
    '''Matching strategy that avoids matching side utterances.'''

    @staticmethod
    def find_all(words, utterance):
        '''
        Finds all indexes of words in utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            [int]: indexes
        '''
        indexes = []
        start = 0
        res = utterance.find(words, start)
        while res > -1:
            indexes += [res]
            start = res + 1
            res = utterance.find(words, start)
        return indexes

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance, avoiding side
        utterances.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        bad_follow_words = ['hand', 'arm']
        # Get basic test out of the way.
        if words not in utterance:
            return False

        # At this point, the words are in the utterance. Check whether
        # we're really matching the phrase...
        indexes = NotSideMatcher.find_all(words, utterance)

        for index in indexes:
            nextidx = index + len(words) + 1  # + 1 for space
            # If the next word is 'hand' or 'arm', we've actually
            # matched one of the side phrases.
            if nextidx < len(utterance):
                for bw in bad_follow_words:
                    if utterance[nextidx:nextidx + len(bw)] == bw:
                        return False

        # OK!
        return True


class Matchers:
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'notside': NotSideMatcher
    }


class WorldObject:
    '''
    Contains data about the properties of an object in the world.

    Can be robot or YAML-loaded.
    '''

    def __init__(self):
        pass

    @staticmethod
    def from_yaml(objs):
        '''
        Args:
            objs (dict): YAML-loaded 'objects' component of world dict.
        '''
        wobjs = []
        for obj in objs:
            wobj = WorldObject()
            wobj.properties = obj
            wobjs += [wobj]
        return wobjs


########################################################################
# Main
########################################################################

def main():
    # Display settings.
    display_limit = 20

    # Load
    command_dict = CommandDict(yaml.load(open(COMMAND_GRAMMAR)))

    world_dict = yaml.load(open(WORLD))
    w_objects = WorldObject.from_yaml(world_dict['objects'])
    robot_dict = world_dict['robot']
    # w_objects = []  # For testing no objects.

    # Make templates (this extracts options and params).
    templates = command_dict.make_templates(w_objects)
    Debug.p('Templates:')
    for t in templates:
        Debug.pl(1, t)
    Info.p("Templates: " + str(len(templates)))

    # Make commands
    commands = [ct.generate_commands() for ct in templates]
    commands = [i for s in commands for i in s]  # Flatten.
    Info.p("Commands: " + str(len(commands)))

    # Make sentences
    sentences = [c.generate_sentences() for c in commands]
    sentences = [i for s in sentences for i in s]  # Flatten.
    Info.p("Sentences: " + str(len(sentences)))

    # Apply W and R to weight C prior.
    for c in commands:
        c.apply_w()
        c.apply_r(robot_dict)
    Util.normalize(commands)

    # Pre-score commands with all possible sentences.
    for c in commands:
        c.score_match_sentences(sentences)

    # Display commands.
    commands = sorted(commands, key=lambda x: -x.score)
    ptot = 0.0
    Info.p("Top commands (before utterance):")
    for idx, c in enumerate(commands):
        if idx < display_limit:
            Info.pl(1, c)
        ptot += c.score
    Info.p('Total (%d): %0.2f' % (len(commands), ptot))

    # Get user input here (normally).
    utterance = 'move right hand up'
    Info.p("Utterance: " + utterance)

    # Score sentences.
    for s in sentences:
        s.score_match(utterance)
    Util.normalize(sentences)

    # Apply L.
    Command.apply_l_precheck(commands, sentences)
    for c in commands:
        c.apply_l(sentences)
    Util.normalize(commands, 'lang_score', 0.0)

    # Get combined probability.
    for c in commands:
        c.get_final_p()
    Util.normalize(commands, 'final_p', 0.0)

    # Display sentences.
    sentences = sorted(sentences, key=lambda x: -x.score)
    Info.p('Top sentences:')
    for idx, s in enumerate(sentences):
        if idx < display_limit:
            Info.pl(1, s)

    # Display commands.
    commands = sorted(commands, key=lambda x: -x.final_p)
    ptot = 0.0
    Info.p("Top commands:")
    for idx, c in enumerate(commands):
        if idx < display_limit:
            Info.pl(1, c)
        ptot += c.final_p
    Info.p('Total (%d): %0.2f' % (len(commands), ptot))

if __name__ == '__main__':
    main()
