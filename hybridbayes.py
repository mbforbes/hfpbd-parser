'''Hybrid approach: Bayes conceptually, score functions in low-level.'''

########################################################################
# Imports
########################################################################

# Builtins
import code
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
DEBUG_PRINTING_DEFAULT = False

COMMAND_GRAMMAR = 'commands.yml'
WORLD = 'world.yml'
VOCAB = 'vocab.yml'
OBJ_COMPONENT = 'obj'  # We expand this component by name.

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

# Score-related
START_SCORE = 0.0  # To begin with. Maybe doesn't matter.
MIN_SCORE = 10.0  # Minimum score for normalizing. Only adds to match.
P_NOOBJ = -80  # Penalty: there's no object and the score requires one.
P_LOCUNR = -50  # Penalty: the requested location is unreachable.
P_OBJUNR = -50  # Penalty: the requested object cannot be picked up.
P_NOTLASTSIDE = -10  # Penalty: the side wasn't the last commanded.
P_GSTATE = -80  # Penalty: nonsensical gripper state change requested.
P_BADPP = -50  # Penalty: bad pickup/place command requested (from GS)


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





class Command:
    '''A fully-instantiated command.

    It starts as a shell when initialized, but adding to self.params
    causes the command to become fully-instantiated.
    '''

    longest_name_len = 0

    def __init__(self, name):
        '''
        Args:
            name (str)
        '''
        self.name = name
        self.params = {}
        self.score = START_SCORE

        # Track longest name for uniform logging
        l = len(self.name)
        pl = Command.longest_name_len
        Command.longest_name_len = l if l > pl else pl

    def __str__(self):
        lnl = Command.longest_name_len
        score_str = "%0.3f" % (self.score)
        padding = 0 if len(self.name) == lnl else lnl - len(self.name)
        name_str = self.name + ' ' * padding
        param_str = ', '.join(
            [': '.join([str(k), str(v)]) for k, v in self.params.iteritems()])
        return "%s   %s   %s" % (score_str, name_str, param_str)


########################################################################
# Functions
########################################################################

def generate_all_commands(commands, objects={}):
    '''
    Args:
        commands (dict): YAML-loaded
        objects (dict)

    Returns:
        [Command]: Commands
    '''
    # Pre-extract object options.
    objs = [o['name'] for o in objects]
    objs = [None] if len(objs) == 0 else objs

    # Now, generate commands.
    all_results = []
    for cmd, components in commands['commands'].iteritems():
        Debug.p(cmd)
        cmd_results = [Command(cmd)]
        # Sometimes there are no parameters to expand.
        if components is not None:
            # Otherwise, expand all components.
            for component in components:
                Debug.pl(1, component)
                # All options must be appended separately.
                next_cmd_results = []
                if component == OBJ_COMPONENT:
                    options = objs
                else:
                    options = commands['options'][component]
                for opt in options:
                    Debug.pl(2, opt)
                    for prev_cmd in cmd_results:
                        new_cmd = copy.deepcopy(prev_cmd)
                        new_cmd.params[component] = opt
                        next_cmd_results += [new_cmd]

                # Append on the results of this component expansion.
                cmd_results = next_cmd_results
        # Append on this command
        # += [cmd_results] groups list by command; could be useful...
        all_results += cmd_results
    # Display
    Debug.p('number of commands: ' + len(all_results))
    return all_results


def c_apply_w(commands, objects):
    '''
    Applies world (objects) influence to command scores.

    Negatively affect commands that contain objects referring to
    'impossible configurations.'

    - lookat: obj exist?
    - move_rel: move obj to relpos w/ side possible?
    - place_rel: same as move_rel
    - pickup: pickup obj w/ side possible?

    Args:
        commands ([Command])
        objects (dict)
    '''
    for c in commands:
        # lookat: check object existing
        if c.name == 'lookat':
            if len(objects) == 0:
                # No objs: receive that negative score.
                c.score += P_NOOBJ

        # move_rel, place: check relpos, obj, side
        if c.name == 'move_rel' or c.name == 'place':
            if len(objects) == 0:
                # No objs: receive that negative score.
                c.score += P_NOOBJ
            else:
                # Assuming object names are unique, there should be
                # exactly one.
                obj = [o for o in objects if o['name'] == c.params['obj']][0]
                relpos = c.params['relpos']
                side = c.params['side']
                loc_reachable = obj[M_OP[relpos]][S[side]]
                if not loc_reachable:
                    c.score += P_LOCUNR

        # pickup: check obj, side
        if c.name == 'pickup':
            if len(objects) == 0:
                # No objs: receive that negative score.
                c.score += P_NOOBJ
            else:
                # Assuming object names are unique, there should be
                # exactly one.
                obj = [o for o in objects if o['name'] == c.params['obj']][0]
                side = c.params['side']
                loc_reachable = obj['is_pickupable'][S[side]]
                if not loc_reachable:
                    c.score += P_OBJUNR


def c_apply_r(commands, robot):
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
    for c in commands:
        pks = c.params.keys()

        # Side: last moved? Less likely down.
        if 'side' in pks:
            last_cmd_side = robot['last_cmd_side']
            if (last_cmd_side != 'neither' and
                    c.params['side'] != last_cmd_side):
                c.score += P_NOTLASTSIDE

        # Open/closed & pickup/place: Basd on state (impossible down).
        if c.name == 'open':
            gs = robot['gripper_states'][S[c.params['side']]]
            if gs == 'open':
                c.score += P_GSTATE
        if c.name == 'close':
            gs = robot['gripper_states'][S[c.params['side']]]
            if gs == 'closed_empty' or gs == 'has_obj':
                c.score += P_GSTATE
        if c.name == 'pickup':
            gs = robot['gripper_states'][S[c.params['side']]]
            if gs == 'has_obj':
                c.score += P_BADPP
        if c.name == 'place':
            gs = robot['gripper_states'][S[c.params['side']]]
            if gs == 'open' or gs == 'closed_empty':
                c.score += P_BADPP

        # move_abs: absdir possible to move to
        if c.name == 'move_abs':
            side = c.params['side']
            loc_reachable = robot[M_RP[c.params['absdir']]][S[side]]
            if not loc_reachable:
                c.score += P_LOCUNR


def generate_all_sentences(commands, objects={}):
    '''
    Args:
        commands ([Command])
        vocab (dict): YAML-loaded

    Returns:
        [Sentence]
    '''
    pass


def l_apply_w(sentences, objects):
    '''
    Applies world (objects) influence to language scores.

    Args:
        sentences ([Sentence])
        objects (dict)
    '''
    pass


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


def score(commands_dict, objects, robot):
    '''
    Args:
        commands_dict (dict): Direct, YAML-loaded commands dictionary.
        objects (dict): objects map (e.g. in world YAML file)
        robot (dict): robot map (e.g. in world YAML file)

    Returns:
        Command: top-scoring command
    '''
    commands = generate_all_commands(commands_dict, objects)

    # Calc P(C|W,R)
    c_apply_w(commands, objects)
    c_apply_r(commands, robot)
    normalize(commands)

    # Calc P(L|W,R,C)
    # TODO: all this

    # Display: highest first (hence - score comparison)
    commands = sorted(commands, key=lambda x: -x.score)
    for c in commands:
        print c
    print '-- Total:', sum([c.score for c in commands])
    return commands[0]


def check_vocab(vocab_dict, commands_dict):
    '''
    Ensures vocab entires are comprehensive of commands_dict entries.

    Both sets of commands should match exactly.

    The components of vocab should be a superset of the components of
    commands.

    Args:
        commands_dict (dict): Direct, YAML-loaded commands dictionary.
        vocab_dict (dict): Direct, YAML-loaded vocab dictionary.

    '''
    failed = False
    # Check all in vocab are in commands.
    for cmd, components in vocab_dict['vocab']['commands'].iteritems():
        if cmd not in commands_dict['commands']:
            failed = True
            Error.p('Command ' + cmd + ' not in commands dict.')

    # Check commands AND components in commands are in vocab.
    for cmd, components in commands_dict['commands'].iteritems():
        if cmd not in vocab_dict['vocab']['commands']:
            failed = True
            Error.p('Command ' + cmd + ' not in vocab dict.')
        else:
            # We have the command; make sure all components in commands
            # are in vocab.
            for component in components:
                if component not in vocab_dict['vocab']['commands'][cmd]:
                    failed = True
                    Error.p(
                        'Component ' + component + ' found in commands dict ' +
                        cmd + ' but not in vocab dict.')

    # We really want to make sure these match.
    if failed:
        sys.exit(1)


########################################################################
# Main
########################################################################

def main():
    # Load
    commands_dict = yaml.load(open(COMMAND_GRAMMAR))

    world_dict = yaml.load(open(WORLD))
    objects = world_dict['objects']
    robot = world_dict['robot']
    # objects = {}  # For testing no objects.

    vocab_dict = yaml.load(open(VOCAB))

    # Check stuff
    check_vocab(vocab_dict, commands_dict)

    # Do stuff
    # code.interact(local=locals())
    score(commands_dict, objects, robot)

if __name__ == '__main__':
    main()
