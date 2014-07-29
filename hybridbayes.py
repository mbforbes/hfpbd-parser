'''Hybrid approach: Bayes conceptually, score functions in low-level.'''

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
ERROR_PRINTING_DEFAULT = True
INFO_PRINTING_DEFAULT = True
DEBUG_PRINTING_DEFAULT = True

COMMAND_GRAMMAR = 'commands.yml'
WORLD = 'world.yml'
OBJ_PARAM = 'obj'

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
LANG_UNMATCH_SCORE = 10


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

        Debug.p('Templates:')
        for t in templates:
            Debug.p(t)
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
                matcher = props['strategy']

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
        param_copy = copy.deepcopy(self.params)
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

    def __init__(self, name, option_map, template):
        '''
        Args:
            name (str)
            option_map ([{str: Option}])
            template (CommandTemplate)
        '''
        self.name = name
        self.option_map = option_map
        self.template = template
        self.score = START_SCORE

    def __repr__(self):
        '''
        Returns:
            str
        '''
        lnl = CommandDict.longest_cmd_name_len
        score_str = "%0.3f" % (self.score)
        padding = 0 if len(self.name) == lnl else lnl - len(self.name)
        name_str = self.name + ' ' * padding
        opt_str = ', '.join(
            [': '.join(
                [str(k), str(v)]) for k, v in self.option_map.iteritems()])
        return "%s   %s   %s" % (score_str, name_str, opt_str)

    @staticmethod
    def normalize(commands):
        '''
        Normalizes command scores to a valid probability distribution.

        Args:
            commands ([Command])
        '''
        # First, boost all to some minimum value.
        min_ = min([c.score for c in commands])
        boost = MIN_SCORE - min_ if MIN_SCORE > min_ else 0.0
        for c in commands:
            c.score = c.score + boost

        # Next, do the normalization.
        sum_ = sum([c.score for c in commands])
        for c in commands:
            c.score = c.score / sum_

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


class Sentence:
    '''Our representation of a perfect 'utterance'.

    A series of phrases.'''

    def __init__(self, phrases):
        self.phrases = phrases


class Phrase:
    '''Holds a set of words and a matching strategy for determining if
    it is matched in an utterance.'''

    def __init__(self, words, strategy):
        self.words = words
        self.strategy = strategy

    def score(self, utterance):
        '''
        Args:
            utterance (str)

        Returns:
            int
        '''
        if strategy.match(self.words, utterance):
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

    # def get_words(self):
    #     '''Returns the phrase's words.

    #     Returns:
    #         str
    #     '''
    #     return self.words


class MatchingStrategy:
    '''Interface for matching strategies.'''

    @classmethod
    def match(words, utterance):
        Error.p("MatchingStrategy:match unimplemented as it's an interface.")
        sys.exit(1)


class DefaultMatcher:
    '''Default matching strategy.'''

    @classmethod
    def match(words, utterance):
        '''Returns whether words match an utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        return words in utterance


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
    # Load
    command_dict = CommandDict(yaml.load(open(COMMAND_GRAMMAR)))

    world_dict = yaml.load(open(WORLD))
    w_objects = WorldObject.from_yaml(world_dict['objects'])
    robot_dict = world_dict['robot']
    # w_objects = []  # For testing no objects.

    templates = command_dict.make_templates(w_objects)
    commands = [ct.generate_commands() for ct in templates]
    commands = [i for s in commands for i in s]  # Flatten.

    # Do scoring
    for c in commands:
        c.apply_w()
        c.apply_r(robot_dict)
    Command.normalize(commands)
    commands = sorted(commands, key=lambda x: -x.score)

    ptot = 0.0
    for c in commands:
        Debug.p(c)
        ptot += c.score
    Debug.p('Total (%d): %0.2f' % (len(commands), ptot))

    # OLD
    # ---
    # Check stuff
    # check_vocab(commands_dict)
    # generate_all_sentences(commands_dict)

    # Do stuff
    # code.interact(local=locals())
    # score(commands_dict, objects, robot)

if __name__ == '__main__':
    main()
