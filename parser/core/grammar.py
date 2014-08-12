'''Grammar extraction / generation.

    - Extract all Phrases

    - Use the Phrases to build Options
        - WordOptions' phrases come from a template (grammar file)
        - ObjectOptions' phrases come from the object properties
            (objects are loaded into WorldObjects), which then refer to
            WordOptions.

    - Use the Options to build Parameters

    - Use the Paramters to build CommandTemplates

    - Use the CommandTemplates to generate all Commands
'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
from collections import OrderedDict, defaultdict
import copy

# Local
from constants import C, N
from matchers import DefaultMatcher, MatchingStrategy, Matchers
from util import Logger, Error, Info, Debug, Algo, Numbers


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
            if pname == C.obj_param:
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
        self.score = N.START_SCORE

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
            if wobj.has_property(C.m_op[relpos]):
                reachables = wobj.get_property(C.m_op[relpos])
                if side in C.sides:
                    loc_reachable = reachables[C.side_to_idx[side]]
                else:
                    # Side == both
                    loc_reachable = reachables[0] and reachables[1]
                if not loc_reachable:
                    self.score += N.P_LOCUNR

        # cmds: pickup (params: side)
        if self.name in ['pick_up']:
            side = self.option_map['side'].name
            # NOTE(mbforbes): What do we assume if there's no such
            # property? I'll assume no informations means do not affect
            # the score.
            if wobj.has_property('is_pickupable'):
                reachables = wobj.get_property('is_pickupable')
                if side in C.sides:
                    loc_reachable = reachables[C.side_to_idx[side]]
                else:
                    # Side == both
                    loc_reachable = reachables[0] and reachables[1]
                if not loc_reachable:
                    self.score += N.P_OBJUNR

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
                self.score += N.P_DUMBSTOP
            elif self.name == 'execute' and is_exec:
                self.score += N.P_DUMBEXEC

        # All following robot applications involve a side.
        if not self.template.has_params(['side']):
            return

        cmd_side = self.option_map['side'].name
        side_idx = C.side_to_idx[cmd_side]

        # Side: last moved? Less likely down.
        if robot.has_property('last_cmd_side'):
            last_cmd_side = robot.get_property('last_cmd_side')
            if (last_cmd_side != 'neither' and
                    self.option_map['side'].name != last_cmd_side):
                self.score += N.P_NOTLASTSIDE

        # Open/closed & pickup/place: Basd on gripper state (impossible
        # configurations have scores lowered).
        if robot.has_property('gripper_states'):
            gs = robot.get_property('gripper_states')[side_idx]
            if self.name == 'open':
                if gs == 'open':
                    self.score += N.P_GSTATE
            elif self.name == 'close':
                if gs == 'closed_empty' or gs == 'has_obj':
                    self.score += N.P_GSTATE
            elif self.name == 'pick_up':
                if gs == 'has_obj':
                    self.score += N.P_BADPP
            elif self.name == 'place':
                if gs == 'open' or gs == 'closed_empty':
                    self.score += N.P_BADPP

        # move_abs: absdir possible to move to
        if self.name == 'move_abs':
            prop_name = C.m_rp[self.option_map['abs_dir'].name]
            if robot.has_property(prop_name):
                loc_reachable = robot.get_property(prop_name)[side_idx]
                if not loc_reachable:
                    self.score += N.P_LOCUNR

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

    @staticmethod
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
                cmd1.lang_score, cmd2.lang_score, N.LANGUAGE_TIE_EPSILON):
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
        for object_property, word_option_name in C.m_wo.iteritems():
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
