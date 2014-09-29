'''We need to keep some things in sync with the grammar file
(commands.yml). However, putting a ton of meta info there will make it
harder to add (though it won't break things elsewhere).

For now, we're using this file as a link between the code and the
grammar file. Ideally, we'll phase the strings out of this file and have
them automatically extracted from the grammar file.
'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
from collections import OrderedDict
import yaml

# Local
from util import Fs


########################################################################
# Module-level constants (internal to this file!)
########################################################################

DATA_DIR = Fs.data_dir()
COMMAND_GRAMMAR = 'commands.yml'
WORLD_DEFAULT = 'world_default.yml'
SIDES = yaml.load(open(DATA_DIR + COMMAND_GRAMMAR))['parameters']['side']
sm = {}
for idx, side_name in enumerate(SIDES):
    sm[side_name] = idx


########################################################################
# Classes
########################################################################

class N(object):
    # Command scoring.
    MIN_SCORE = 0.1  # What to boost scores to *before* normalizing.
    START_SCORE = 1.0  # To begin with. Maybe doesn't matter.
    P_LOCUNR = -5.0  # Penalty: the requested location is unreachable.
    P_OBJUNR = -5.0  # Penalty: the requested object cannot be picked up.
    P_NOTLASTSIDE = -0.1  # Penalty: the side wasn't the last commanded.
    P_NOTLASTREFOBJ = -0.1  # Penalty: the obj wasn't the last referred.
    P_GSTATE = -8.0  # Penalty: nonsensical gripper state change requested.
    P_DUMBSTOP = -10.0  # Penalty: can't stop when not executing.
    P_DUMBEXEC = -10.0  # Penalty: can't execute when already doing so.
    P_BADPP = -5.0  # Penalty: bad pickup/place command requested (from GS)

    # How close language scores have to be before we start to use the
    # command score (world and robot-influenced) to break ties. This
    # really needs to be tuned as the number of comamnds (sentences)
    # grows.
    LANGUAGE_TIE_EPSILON = 0.001


class C(object):
    '''C for constants (duh)--these are mappings computed from other
    constants.'''

    command_grammar = DATA_DIR + COMMAND_GRAMMAR
    world_default = DATA_DIR + WORLD_DEFAULT

    # Grammar file constants
    obj_param = 'obj'

    # Side names
    sides = SIDES

    # How to index into object properties by side
    side_to_idx = sm

    # Object constants
    # ---------------
    # Map (M) from component options to object properties (OP).
    # The type of values for these, when set, are bool[]s.
    m_op = {
        # abs
        'above': 'is_above_reachable',
        'next_to': 'is_nextto_reachable',
        'to_left_of': 'is_leftof_reachable',
        'to_right_of': 'is_rightof_reachable',
        'in_front_of': 'is_frontof_reachable',
        'behind': 'is_behind_reachable',
        'on_top_of': 'is_topof_reachable',
        'near': 'is_near_reachable',
        # rel
        'towards': 'is_towards_reachable',
        'away': 'is_away_reachable',
    }

    # This sets where color (the identifying adjective) goes in the
    # priority list of descriptions. List all adjective properties
    # here that go BEFORE color.
    color_priority = [
        'left_most',
        'right_most',
        'middle',
        'farthest',
        'nearest',
        'biggest',
        'smallest',
        'tallest',
        'shortest',
    ]

    # Map (M) from object properties to WordOption names (WO).
    # The type of the values for these, when set, are bools.
    # NOTE(mbforbes): The ordering here sets the priority of object
    # descriptions.
    m_wo = OrderedDict([
        ('is_leftmost', 'left_most'),
        ('is_rightmost', 'right_most'),

        ('is_middle', 'middle'),

        ('is_farthest', 'farthest'),
        ('is_nearest', 'nearest'),

        ('is_biggest', 'biggest'),
        ('is_smallest', 'smallest'),

        ('is_tallest', 'tallest'),
        ('is_shortest', 'shortest'),

        # Color goes here; keep color_priority (above) up-to-date if
        # this changes.
    ])
    m_wo_inverse = {v: k for k, v in m_wo.iteritems()}

    # Object properties that are referred to by the same name in ROS objects
    # as well as the parser.
    # The tope of the values for these, when set, are strs.
    op_strs = [
        'name',
        'color',
        'type',
    ]

    # These, like those in M_OP, are bool[]s, but these don't require
    # translating from component options to object properties because they
    # are always accessed directly by object property name.
    op_boolarrs = [
        'is_pickupable',
    ]

    # Robot constants
    # ----------------
    # Map (M) from component options to robot properties (RP).
    # The values of these are typed bool[].
    m_rp = {
        'up': 'can_move_up',
        'down': 'can_move_down',
        'to_left': 'can_move_toleft',
        'to_right': 'can_move_toright',
        'forward': 'can_move_forward',
        'backward': 'can_move_backward',
    }

    # Robot properties whose vals are typed str.
    rp_strs = [
        'last_cmd_side',
        'last_referred_obj_name',
    ]

    # Robot properties whose vals are typed bool[].
    rp_bool_arrs = [
        'gripper_states',
    ]

    # Robot properties whose vals are typed bool.
    rp_bools = [
        'is_executing',
    ]
