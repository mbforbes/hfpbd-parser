'''Tests for hybridbayes hfpbd-parser.

Current test categories include:
    - Matcher: matcher tests (phrase matching)
    - Full: end-to-end tests (of entire system)

Tests for Full include
    [x] Action 1: move ( abs_pose , arm_side )
    [x] Action 2: move ( rel_pose , object , arm_side )
    [x] Action 3: move ( abs_direction , arm_side )
    [x] Action 4: move ( rel_direction , object , arm_side )
    [ ] Action 5: place ( abs_loc , arm_side )
    [x] Action 6: place ( rel_pose , object , arm_side )
    [x] Action 7: pick_up ( object , arm_side )
    [ ] Action 8: point_to ( object , arm_side )
    [ ] Action 9: rotate ( joint , rotation_direction )
    [x] Action 10: look_at ( object )
    [x] Action 11: open ( arm_side )
    [x] Action 12: close ( arm_side )

    Plus 'admin' commands:
    [ ] Create new action
    [ ] Switch to action ( which )
    [ ] Execute
    [ ] Stop
'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Builtins
import getpass
import unittest

# Local
from parser.core.frontends import Frontend
from parser.core.roslink import WorldObject, Robot, RobotCommand
from parser.core.util import Info, Debug
from parser.core.matchers import DefaultMatcher


# ######################################################################
# Constants (will have to keep sync'd with grammar file)
# ######################################################################

# Strings (S)
S_OPEN_CLOSE = {
    'OPENRIGHT': 'open right-hand',
    'OPENLEFT': 'open left-hand',
    'CLOSERIGHT': 'close right-hand',
    'CLOSELEFT': 'close left-hand',
}
S_MOVEABSPOS = {
    'LH_TOSIDE': 'move left-hand to-the-side',
    'RH_TOSIDE': 'move right-hand to-the-side',
}
S_MOVEABS = {
    'LH_UP': 'move left-hand up',
    'LH_DOWN': 'move left-hand down',
    'LH_LEFT': 'move left-hand left',
    'LH_RIGHT': 'move left-hand right',
    'LH_FORWARD': 'move left-hand forward',
    'LH_BACKWARD': 'move left-hand backwards',

    'RH_UP': 'move right-hand up',
    'RH_DOWN': 'move right-hand down',
    'RH_LEFT': 'move right-hand left',
    'RH_RIGHT': 'move right-hand right',
    'RH_FORWARD': 'move right-hand forward',
    'RH_BACKWARD': 'move right-hand backwards',
}
S_MOVEREL = {
    'RH_ABOVE': 'move right-hand above the red box',
    'RH_NEXTTO': 'move right-hand next-to the red box',
    'RH_LEFTOF': 'move right-hand to-the-left-of the red box',
    'RH_RIGHTOF': 'move right-hand to-the-right-of the red box',
    'RH_FRONTOF': 'move right-hand in-front-of the red box',
    'RH_BEHIND': 'move right-hand behind the red box',
    'RH_TOPOF': 'move right-hand on-top-of the red box',
    'RH_NEAR': 'move right-hand near the red box',

    'LH_ABOVE': 'move left-hand above the red box',
    'LH_NEXTTO': 'move left-hand next-to the red box',
    'LH_LEFTOF': 'move left-hand to-the-left-of the red box',
    'LH_RIGHTOF': 'move left-hand to-the-right-of the red box',
    'LH_FRONTOF': 'move left-hand in-front-of the red box',
    'LH_BEHIND': 'move left-hand behind the red box',
    'LH_TOPOF': 'move left-hand on-top-of the red box',
    'LH_NEAR': 'move left-hand near the red box',
}
S_MOVEREL_DIR = {
    'RH_AWAY': 'move right-hand away-from the red box',
    'RH_TOWARDS': 'move right-hand towards the red box',
    'LH_AWAY': 'move left-hand away-from the red box',
    'LH_TOWARDS': 'move left-hand towards the red box',
}
S_PICKUP = {
    'RH': 'pick-up the red box with your right-hand',
    'LH': 'pick-up the red box with your left-hand',
}
S_PLACELOC = {
    'RH_TABLE': 'place on-the-table with your right-hand',
}
S_PLACE = {
    'PL_RH_ABOVE': 'place above the red box with your right-hand',
    'PL_RH_NEXTTO': 'place next-to the red box with your right-hand',
    'PL_RH_LEFTOF': 'place to-the-left-of the red box with your right-hand',
    'PL_RH_RIGHTOF': 'place to-the-right-of the red box with your right-hand',
    'PL_RH_FRONTOF': 'place in-front-of the red box with your right-hand',
    'PL_RH_BEHIND': 'place behind the red box with your right-hand',
    'PL_RH_TOPOF': 'place on-top-of the red box with your right-hand',
    'PL_RH_NEAR': 'place near the red box with your right-hand',

    'PL_LH_ABOVE': 'place above the red box with your left-hand',
    'PL_LH_NEXTTO': 'place next-to the red box with your left-hand',
    'PL_LH_LEFTOF': 'place to-the-left-of the red box with your left-hand',
    'PL_LH_RIGHTOF': 'place to-the-right-of the red box with your left-hand',
    'PL_LH_FRONTOF': 'place in-front-of the red box with your left-hand',
    'PL_LH_BEHIND': 'place behind the red box with your left-hand',
    'PL_LH_TOPOF': 'place on-top-of the red box with your left-hand',
    'PL_LH_NEAR': 'place near the red box with your left-hand',

}
S_LOOKAT = {
    'LOOKAT': 'look at the red box',
}

# RobotCommands (RC)
RC_OPEN_CLOSE = {
    'OPENRIGHT': RobotCommand.from_strs('open', ['right_hand']),
    'OPENLEFT': RobotCommand.from_strs('open', ['left_hand']),
    'CLOSERIGHT': RobotCommand.from_strs('close', ['right_hand']),
    'CLOSELEFT': RobotCommand.from_strs('close', ['left_hand']),
}
RC_MOVEABSPOS = {
    'LH_TOSIDE': RobotCommand.from_strs(
        'move_abs_pos', ['left_hand', 'to_side']),
    'RH_TOSIDE': RobotCommand.from_strs(
        'move_abs_pos', ['right_hand', 'to_side']),
}
RC_MOVEABS = {
    'LH_UP': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'up']),
    'LH_DOWN': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'down']),
    'LH_LEFT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_left']),
    'LH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_right']),
    'LH_FORWARD': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'forward']),
    'LH_BACKWARD': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'backward']),
    'RH_UP': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'up']),
    'RH_DOWN': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'down']),
    'RH_LEFT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_left']),
    'RH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_right']),
    'RH_FORWARD': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'forward']),
    'RH_BACKWARD': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'backward']),
}
RC_MOVEREL = {
    # obj0
    'RH_ABOVE': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'above', 'obj0']),
    'RH_NEXTTO': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'next_to', 'obj0']),
    'RH_LEFTOF': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'to_left_of', 'obj0']),
    'RH_RIGHTOF': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'to_right_of', 'obj0']),
    'RH_FRONTOF': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'in_front_of', 'obj0']),
    'RH_BEHIND': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'behind', 'obj0']),
    'RH_TOPOF': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'on_top_of', 'obj0']),
    'RH_NEAR': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'near', 'obj0']),
    'LH_ABOVE': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'above', 'obj0']),
    'LH_NEXTTO': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'next_to', 'obj0']),
    'LH_LEFTOF': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'to_left_of', 'obj0']),
    'LH_RIGHTOF': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'to_right_of', 'obj0']),
    'LH_FRONTOF': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'in_front_of', 'obj0']),
    'LH_BEHIND': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'behind', 'obj0']),
    'LH_TOPOF': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'on_top_of', 'obj0']),
    'LH_NEAR': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'near', 'obj0']),

    # obj1 new
    'RH_ABOVE2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'above', 'obj1']),
    'RH_NEXTTO2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'next_to', 'obj1']),
    'RH_LEFTOF2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'to_left_of', 'obj1']),
    'RH_RIGHTOF2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'to_right_of', 'obj1']),
    'RH_FRONTOF2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'in_front_of', 'obj1']),
    'RH_BEHIND2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'behind', 'obj1']),
    'RH_TOPOF2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'on_top_of', 'obj1']),
    'RH_NEAR2': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'near', 'obj1']),
    'LH_ABOVE2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'above', 'obj1']),
    'LH_NEXTTO2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'next_to', 'obj1']),
    'LH_LEFTOF2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'to_left_of', 'obj1']),
    'LH_RIGHTOF2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'to_right_of', 'obj1']),
    'LH_FRONTOF2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'in_front_of', 'obj1']),
    'LH_BEHIND2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'behind', 'obj1']),
    'LH_TOPOF2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'on_top_of', 'obj1']),
    'LH_NEAR2': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'near', 'obj1']),
}
RC_MOVEREL_DIR = {
    # obj0
    'RH_AWAY': RobotCommand.from_strs(
        'move_rel_dir', ['right_hand', 'away', 'obj0']),
    'RH_TOWARDS': RobotCommand.from_strs(
        'move_rel_dir', ['right_hand', 'towards', 'obj0']),
    'LH_AWAY': RobotCommand.from_strs(
        'move_rel_dir', ['left_hand', 'away', 'obj0']),
    'LH_TOWARDS': RobotCommand.from_strs(
        'move_rel_dir', ['left_hand', 'towards', 'obj0']),

    # obj1
    'RH_AWAY2': RobotCommand.from_strs(
        'move_rel_dir', ['right_hand', 'away', 'obj1']),
    'RH_TOWARDS2': RobotCommand.from_strs(
        'move_rel_dir', ['right_hand', 'towards', 'obj1']),
    'LH_AWAY2': RobotCommand.from_strs(
        'move_rel_dir', ['left_hand', 'away', 'obj1']),
    'LH_TOWARDS2': RobotCommand.from_strs(
        'move_rel_dir', ['left_hand', 'towards', 'obj1']),
}

RC_PICKUP = {
    # obj0
    'RH': RobotCommand.from_strs(
        'pick_up', ['obj0', 'right_hand']),
    'LH': RobotCommand.from_strs(
        'pick_up', ['obj0', 'left_hand']),
    # obj1
    'RH2': RobotCommand.from_strs(
        'pick_up', ['obj1', 'right_hand']),
    'LH2': RobotCommand.from_strs(
        'pick_up', ['obj1', 'left_hand']),

}
RC_PLACE = {
    # obj0
    'RH_ABOVE': RobotCommand.from_strs(
        'place', ['above', 'obj0', 'right_hand']),
    'RH_NEXTTO': RobotCommand.from_strs(
        'place', ['next_to', 'obj0', 'right_hand']),
    'RH_LEFTOF': RobotCommand.from_strs(
        'place', ['to_left_of', 'obj0', 'right_hand']),
    'RH_RIGHTOF': RobotCommand.from_strs(
        'place', ['to_right_of', 'obj0', 'right_hand']),
    'RH_FRONTOF': RobotCommand.from_strs(
        'place', ['in_front_of', 'obj0', 'right_hand']),
    'RH_BEHIND': RobotCommand.from_strs(
        'place', ['behind', 'obj0', 'right_hand']),
    'RH_TOPOF': RobotCommand.from_strs(
        'place', ['on_top_of', 'obj0', 'right_hand']),
    'RH_NEAR': RobotCommand.from_strs(
        'place', ['near', 'obj0', 'right_hand']),

    'LH_ABOVE': RobotCommand.from_strs(
        'place', ['above', 'obj0', 'left_hand']),
    'LH_NEXTTO': RobotCommand.from_strs(
        'place', ['next_to', 'obj0', 'left_hand']),
    'LH_LEFTOF': RobotCommand.from_strs(
        'place', ['to_left_of', 'obj0', 'left_hand']),
    'LH_RIGHTOF': RobotCommand.from_strs(
        'place', ['to_right_of', 'obj0', 'left_hand']),
    'LH_FRONTOF': RobotCommand.from_strs(
        'place', ['in_front_of', 'obj0', 'left_hand']),
    'LH_BEHIND': RobotCommand.from_strs(
        'place', ['behind', 'obj0', 'left_hand']),
    'LH_TOPOF': RobotCommand.from_strs(
        'place', ['on_top_of', 'obj0', 'left_hand']),
    'LH_NEAR': RobotCommand.from_strs(
        'place', ['near', 'obj0', 'left_hand']),


    # new obj1
    'RH_ABOVE2': RobotCommand.from_strs(
        'place', ['above', 'obj1', 'right_hand']),
    'RH_NEXTTO2': RobotCommand.from_strs(
        'place', ['next_to', 'obj1', 'right_hand']),
    'RH_LEFTOF2': RobotCommand.from_strs(
        'place', ['to_left_of', 'obj1', 'right_hand']),
    'RH_RIGHTOF2': RobotCommand.from_strs(
        'place', ['to_right_of', 'obj1', 'right_hand']),
    'RH_FRONTOF2': RobotCommand.from_strs(
        'place', ['in_front_of', 'obj1', 'right_hand']),
    '_RH_BEHIND2': RobotCommand.from_strs(
        'place', ['behind', 'obj1', 'right_hand']),
    'RH_TOPOF2': RobotCommand.from_strs(
        'place', ['on_top_of', 'obj1', 'right_hand']),
    'RH_NEAR2': RobotCommand.from_strs(
        'place', ['near', 'obj1', 'right_hand']),

    'LH_ABOVE2': RobotCommand.from_strs(
        'place', ['above', 'obj1', 'left_hand']),
    'LH_NEXTTO2': RobotCommand.from_strs(
        'place', ['next_to', 'obj1', 'left_hand']),
    'LH_LEFTOF2': RobotCommand.from_strs(
        'place', ['to_left_of', 'obj1', 'left_hand']),
    'LH_RIGHTOF2': RobotCommand.from_strs(
        'place', ['to_right_of', 'obj1', 'left_hand']),
    'LH_FRONTOF2': RobotCommand.from_strs(
        'place', ['in_front_of', 'obj1', 'left_hand']),
    'LH_BEHIND2': RobotCommand.from_strs(
        'place', ['behind', 'obj1', 'left_hand']),
    'LH_TOPOF2': RobotCommand.from_strs(
        'place', ['on_top_of', 'obj1', 'left_hand']),
    'LH_NEAR2': RobotCommand.from_strs(
        'place', ['near', 'obj1', 'left_hand']),
}
RC_LOOKAT = {
    # obj0
    'LOOKAT': RobotCommand.from_strs(
        'look_at', ['obj0']),
    # obj1
    'LOOKAT2': RobotCommand.from_strs(
        'look_at', ['obj1']),
}

# Objects (O)
O_FULL_REACHABLE = {
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
    'is_towards_reachable': [True, True],
    'is_away_reachable': [True, True],
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

# Only right side is 'possible', where applicable. (also, smallest.
O_RIGHT_POSSIBLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [True, False],
    'is_above_reachable': [True, False],
    'is_nextto_reachable': [True, False],
    'is_leftof_reachable': [True, False],
    'is_rightof_reachable': [True, False],
    'is_frontof_reachable': [True, False],
    'is_behind_reachable': [True, False],
    'is_topof_reachable': [True, False],
    'is_near_reachable': [True, False],
    'is_towards_reachable': [True, False],
    'is_away_reachable': [True, False],
    'is_leftmost': False,
    'is_rightmost': True,
    'is_biggest': False,
    'is_smallest': True,
    # E.g. red, blue, green, unknown
    'color': 'red',
    # E.g. cup, box, unknown
    'type': 'box',
}
# Only left side possible where applicable.
O_LEFT_POSSIBLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [False, True],
    'is_above_reachable': [False, True],
    'is_nextto_reachable': [False, True],
    'is_leftof_reachable': [False, True],
    'is_rightof_reachable': [False, True],
    'is_frontof_reachable': [False, True],
    'is_behind_reachable': [False, True],
    'is_topof_reachable': [False, True],
    'is_near_reachable': [False, True],
    'is_towards_reachable': [False, True],
    'is_away_reachable': [False, True],
    # E.g. red, blue, green, unknown
    'color': 'red',
    # E.g. cup, box, unknown
    'type': 'box',
}
# Has different name than first left-possible object. (also, biggest)
O_LEFT_POSSIBLE_SECOND = {
    'name': 'obj1',
    # Relation to arms.
    'is_pickupable': [False, True],
    'is_above_reachable': [False, True],
    'is_nextto_reachable': [False, True],
    'is_leftof_reachable': [False, True],
    'is_rightof_reachable': [False, True],
    'is_frontof_reachable': [False, True],
    'is_behind_reachable': [False, True],
    'is_topof_reachable': [False, True],
    'is_near_reachable': [False, True],
    'is_towards_reachable': [False, True],
    'is_away_reachable': [False, True],
    'is_leftmost': True,
    'is_rightmost': False,
    'is_biggest': True,
    'is_smallest': False,
    # E.g. red, blue, green, unknown
    'color': 'red',
    # E.g. cup, box, unknown
    'type': 'box',
}
# Has different properties (name, color, type) than the first fully-
# reachable object.
O_FULL_REACHABLE_SECOND = {
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
    'is_towards_reachable': [True, True],
    'is_away_reachable': [True, True],
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
# Noting can reach it!
O_IMPOSSIBLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [False, False],
    'is_above_reachable': [False, False],
    'is_nextto_reachable': [False, False],
    'is_leftof_reachable': [False, False],
    'is_rightof_reachable': [False, False],
    'is_frontof_reachable': [False, False],
    'is_behind_reachable': [False, False],
    'is_topof_reachable': [False, False],
    'is_near_reachable': [False, False],
    'is_towards_reachable': [False, False],
    'is_away_reachable': [False, False],
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

# Robot properties (R)
R_RIGHT_PREF = {
    'last_cmd_side': 'right_hand'
}
R_LEFT_PREF = {
    'last_cmd_side': 'left_hand'
}

R_ONLY_RIGHT_POSSIBLE = {
    'can_move_up': [True, False],
    'can_move_down': [True, False],
    'can_move_toleft': [True, False],
    'can_move_toright': [True, False],
    'can_move_forward': [True, False],
    'can_move_backward': [True, False],
}
R_ONLY_LEFT_POSSIBLE = {
    'can_move_up': [False, True],
    'can_move_down': [False, True],
    'can_move_toleft': [False, True],
    'can_move_toright': [False, True],
    'can_move_forward': [False, True],
    'can_move_backward': [False, True],
}
R_NEITHER_POSSIBLE = {
    'can_move_up': [False, False],
    'can_move_down': [False, False],
    'can_move_toleft': [False, False],
    'can_move_toright': [False, False],
    'can_move_forward': [False, False],
    'can_move_backward': [False, False],
}


# ######################################################################
# Classes
# ######################################################################

class TestUtil:
    @staticmethod
    def on_travis():
        '''
        Returns:
            bool: Whether we're running on Travis-CI
        '''
        # True "on travis?" test:
        return getpass.getuser() == 'travis'


class TestDefaultMatcher(unittest.TestCase):
    def test_part_of_words(self):
        self.assertEqual(DefaultMatcher.match('up', 'cup'), 0.0)

    def test_basic_params(self):
        self.assertNotEqual(DefaultMatcher.match(
            'down', 'move left-hand down'), 0.0)

    def test_side_unmatches(self):
        self.assertEqual(DefaultMatcher.match('right', 'right-hand'), 0.0)

    def test_side_matches(self):
        self.assertNotEqual(DefaultMatcher.match(
            'right-hand', 'right-hand'), 0.0)
        self.assertNotEqual(DefaultMatcher.match(
            'right', 'right-hand right'), 0.0)


class FullNoContext(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        self.frontend.set_world()

    def test_openclose(self):
        for cmd in S_OPEN_CLOSE.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_OPEN_CLOSE[cmd]), RC_OPEN_CLOSE[cmd])

    def test_moveabs(self):
        for cmd in S_MOVEABS.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEABS[cmd]), RC_MOVEABS[cmd])

    def test_moveabspos(self):
        for cmd in S_MOVEABSPOS.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEABSPOS[cmd]), RC_MOVEABSPOS[cmd])


class FullInferOpenClose(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()

    def test_pick_open(self):
        # We're going to weight against the last commanded side.
        robot = Robot({
            'gripper_states': ['closed_empty', 'open'],
            'last_cmd_side': 'left_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('open'),
            RC_OPEN_CLOSE['OPENRIGHT']
        )

        # We're going to weight against the last commanded side.
        robot = Robot({
            'gripper_states': ['open', 'has_obj'],
            'last_cmd_side': 'right_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('release'),
            RC_OPEN_CLOSE['OPENLEFT']
        )

    def test_pick_close(self):
        # We're going to weight against the last commanded side.
        robot = Robot({
            'gripper_states': ['closed_empty', 'open'],
            'last_cmd_side': 'right_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('close'),
            RC_OPEN_CLOSE['CLOSELEFT']
        )

        # We're going to weight against the last commanded side.
        robot = Robot({
            'gripper_states': ['open', 'has_obj'],
            'last_cmd_side': 'left_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('close'),
            RC_OPEN_CLOSE['CLOSERIGHT']
        )


class FullOneObjNoRobot(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [WorldObject(O_FULL_REACHABLE)]
        self.frontend.set_world(world_objects=objs)

    def test_openclose(self):
        for cmd in S_OPEN_CLOSE.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_OPEN_CLOSE[cmd]), RC_OPEN_CLOSE[cmd])

    def test_moveabs(self):
        for cmd in S_MOVEABS.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEABS[cmd]), RC_MOVEABS[cmd])

    def test_moveabspos(self):
        for cmd in S_MOVEABSPOS.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEABSPOS[cmd]), RC_MOVEABSPOS[cmd])

    def test_movereldir(self):
        for cmd in S_MOVEREL_DIR.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEREL_DIR[cmd]), RC_MOVEREL_DIR[cmd])

    def test_moverel(self):
        for cmd in S_MOVEREL.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEREL[cmd]), RC_MOVEREL[cmd])

    def test_pickup(self):
        for cmd in S_PICKUP.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_PICKUP[cmd]), RC_PICKUP[cmd])

    def test_place(self):
        for cmd in S_PLACE.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_PLACE[cmd]), RC_PLACE[cmd])

    def test_lookat(self):
        for cmd in S_LOOKAT.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_LOOKAT[cmd]), RC_LOOKAT[cmd])

    def test_synonyms(self):
        self.assertEqual(
            self.frontend.parse('open left-gripper'),
            RC_OPEN_CLOSE['OPENLEFT'])
        self.assertEqual(
            self.frontend.parse('close right-gripper'),
            RC_OPEN_CLOSE['CLOSERIGHT'])
        self.assertEqual(
            self.frontend.parse('move left-arm higher'),
            RC_MOVEABS['LH_UP'])
        self.assertEqual(
            self.frontend.parse('move right-gripper to-the-left'),
            RC_MOVEABS['RH_LEFT'])
        self.assertEqual(
            self.frontend.parse('move right-gripper away'),
            RC_MOVEABS['RH_FORWARD'])
        self.assertEqual(
            self.frontend.parse('move left-gripper closer'),
            RC_MOVEABS['LH_BACKWARD'])
        self.assertEqual(
            self.frontend.parse('pick the red block up left-hand'),
            RC_PICKUP['LH'])
        self.assertEqual(
            self.frontend.parse('pick the red block up right-arm'),
            RC_PICKUP['RH'])

    def test_missing_words(self):
        self.assertEqual(
            self.frontend.parse('lower left-gripper'),
            RC_MOVEABS['LH_DOWN'])
        self.assertEqual(
            self.frontend.parse('raise right-gripper'),
            RC_MOVEABS['RH_UP'])

    @unittest.skipIf(
        not TestUtil.on_travis(),
        "Test is slow and more of sanity check; OK to run only on Travis.")
    def test_extra_words(self):
        # Collect all strs
        all_strs = {}
        str_items = (
            S_OPEN_CLOSE.items() + S_MOVEABS.items() + S_MOVEREL.items() +
            S_PICKUP.items() + S_PLACE.items() + S_LOOKAT.items())
        for name, words in str_items:
            all_strs[name] = words

        # Collect all RCs
        all_rcs = {}
        rc_items = (
            RC_OPEN_CLOSE.items() + RC_MOVEABS.items() + RC_MOVEREL.items() +
            RC_PICKUP.items() + RC_PLACE.items() + RC_LOOKAT.items())
        for name, rc in rc_items:
            all_rcs[name] = rc

        prefixes = ['Hey rosie', 'Robot, could you', 'Please']
        postfixes = ['thanks.', 'or else.', 'yeah, that would be great.']

        for prefix in prefixes:
            for postfix in postfixes:
                for key, cmd in str_items:
                    query = ' '.join([prefix, cmd, postfix])
                    expected_rc = all_rcs[key]
                    actual_rc = self.frontend.parse(query)[0]
                    self.assertEqual(expected_rc, actual_rc)

    def test_obj_desc(self):
        self.assertEqual(
            self.frontend.parse('pick-up that thing left-hand'),
            RC_PICKUP['LH'])
        self.assertEqual(
            self.frontend.parse('pick-up the box right-hand'),
            RC_PICKUP['RH'])
        self.assertEqual(
            self.frontend.parse('pick-up the red thing right-hand'),
            RC_PICKUP['RH'])


class FullOneObjRobotSidePref(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [WorldObject(O_FULL_REACHABLE)]
        # NOTE: Only 'update objects' called; this ensures we must
        # update the robot in each test method before the parser will
        # run (as we want both to be specified). We could call 'set
        # world' instead but this ensures we write the tests correctly.
        self.frontend.update_objects(world_objects=objs)

    def test_right_preferred(self):
        self.frontend.update_robot(Robot(R_RIGHT_PREF))
        self._check_side_preferred('right_hand')

    def test_left_preferred(self):
        self.frontend.update_robot(Robot(R_LEFT_PREF))
        self._check_side_preferred('left_hand')

    def _check_side_preferred(self, side_rc):
        '''
        Args:
            side_rc: 'right_hand' or 'left_hand'
        '''
        # calculate keys for RC maps
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'

        # open / close
        self.assertEqual(
            self.frontend.parse('open'),
            RC_OPEN_CLOSE['OPEN' + rc_key_long])
        self.assertEqual(
            self.frontend.parse('close'),
            RC_OPEN_CLOSE['CLOSE' + rc_key_long])

        # moveabs
        self.assertEqual(
            self.frontend.parse('move up'),
            RC_MOVEABS[rc_key_short + '_UP'])
        self.assertEqual(
            self.frontend.parse('move down'),
            RC_MOVEABS[rc_key_short + '_DOWN'])
        self.assertEqual(
            self.frontend.parse('move left'),
            RC_MOVEABS[rc_key_short + '_LEFT'])
        self.assertEqual(
            self.frontend.parse('move right'),
            RC_MOVEABS[rc_key_short + '_RIGHT'])
        self.assertEqual(
            self.frontend.parse('move forward'),
            RC_MOVEABS[rc_key_short + '_FORWARD'])
        self.assertEqual(
            self.frontend.parse('move backwards'),
            RC_MOVEABS[rc_key_short + '_BACKWARD'])

        # moveabspos
        self.assertEqual(
            self.frontend.parse('move to-the-side'),
            RC_MOVEABSPOS[rc_key_short + '_TOSIDE'])

        # movereldir
        self.assertEqual(
            self.frontend.parse('move away-from the box'),
            RC_MOVEREL_DIR[rc_key_short + '_AWAY'])
        self.assertEqual(
            self.frontend.parse('move towards the box'),
            RC_MOVEREL_DIR[rc_key_short + '_TOWARDS'])

        # moverel
        self.assertEqual(
            self.frontend.parse('move above the box'),
            RC_MOVEREL[rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('move next-to the box'),
            RC_MOVEREL[rc_key_short + '_NEXTTO'])
        self.assertEqual(
            self.frontend.parse('move to-the-left-of the box'),
            RC_MOVEREL[rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse('move to-the-right-of the box'),
            RC_MOVEREL[rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('move in-front-of the box'),
            RC_MOVEREL[rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('move behind the box'),
            RC_MOVEREL[rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('move on-top-of the box'),
            RC_MOVEREL[rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('move near the box'),
            RC_MOVEREL[rc_key_short + '_NEAR'])

        # pickup, place
        self.assertEqual(
            self.frontend.parse('pick-up'),
            RC_PICKUP[rc_key_short])
        self.assertEqual(
            self.frontend.parse('place above the box'),
            RC_PLACE[rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('place next-to the box'),
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO'])

        self.assertEqual(
            self.frontend.parse('place to-the-left-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse('place to-the-right-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('place in-front-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('place behind the box'),
            RC_PLACE['PL_' + rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('place on-top-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('place near the box'),
            RC_PLACE['PL_' + rc_key_short + '_NEAR'])


class FullRobotOneSidePossible(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()

    def test_only_right_possible(self):
        # Trying with no objects; focus is on ROBOT state.
        robot = Robot(R_ONLY_RIGHT_POSSIBLE)
        self.frontend.set_world(robot=robot)
        self._check_side_possible('right_hand')

    def test_only_left_possible(self):
        # Trying with no objects; focus is on ROBOT state.
        robot = Robot(R_ONLY_LEFT_POSSIBLE)
        self.frontend.set_world(robot=robot)
        self._check_side_possible('left_hand')

    def _check_side_possible(self, side_rc):
        '''
        Args:
            side_rc: 'right_hand' or 'left_hand'
        '''
        # calculate keys for RC maps
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'

        # moveabs
        self.assertEqual(
            self.frontend.parse('move up'),
            RC_MOVEABS[rc_key_short + '_UP'])
        self.assertEqual(
            self.frontend.parse('move down'),
            RC_MOVEABS[rc_key_short + '_DOWN'])
        self.assertEqual(
            self.frontend.parse('move left'),
            RC_MOVEABS[rc_key_short + '_LEFT'])
        self.assertEqual(
            self.frontend.parse('move right'),
            RC_MOVEABS[rc_key_short + '_RIGHT'])
        self.assertEqual(
            self.frontend.parse('move forward'),
            RC_MOVEABS[rc_key_short + '_FORWARD'])
        self.assertEqual(
            self.frontend.parse('move backwards'),
            RC_MOVEABS[rc_key_short + '_BACKWARD'])


class FullObjectOneSidePossible(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()

    def test_only_right_possible(self):
        # Trying with no robot; focus is on OBJECT state.
        self.frontend.set_world(world_objects=[WorldObject(O_RIGHT_POSSIBLE)])
        self._check_object_side('right_hand')

    def test_only_left_possible(self):
        # Trying with no robot; focus is on OBJECT state.
        self.frontend.set_world(world_objects=[WorldObject(O_LEFT_POSSIBLE)])
        self._check_object_side('left_hand')

    def _check_object_side(self, side_rc):
        '''
        Args:
            side_rc: 'right_hand' or 'left_hand'
        '''
        # calculate keys for RC maps
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'

        # moverel
        self.assertEqual(
            self.frontend.parse('move above the red box'),
            RC_MOVEREL[rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('move next-to the red box'),
            RC_MOVEREL[rc_key_short + '_NEXTTO'])
        self.assertEqual(
            self.frontend.parse('move to-the-left-of the box'),
            RC_MOVEREL[rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse('move to-the-right-of the box'),
            RC_MOVEREL[rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('move in-front-of the box'),
            RC_MOVEREL[rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('move behind the box'),
            RC_MOVEREL[rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('move on-top-of the box'),
            RC_MOVEREL[rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('move near the box'),
            RC_MOVEREL[rc_key_short + '_NEAR'])

        # movereldir
        self.assertEqual(
            self.frontend.parse('move away-from the box'),
            RC_MOVEREL_DIR[rc_key_short + '_AWAY'])
        self.assertEqual(
            self.frontend.parse('move towards the box'),
            RC_MOVEREL_DIR[rc_key_short + '_TOWARDS'])

        # pickup
        self.assertEqual(
            self.frontend.parse('pick-up the red box'),
            RC_PICKUP[rc_key_short])

        # place
        self.assertEqual(
            self.frontend.parse('place above the red box'),
            RC_PLACE['PL_' + rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('place next-to the red box'),
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO'])
        self.assertEqual(
            self.frontend.parse('place to-the-left-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse('place to-the-right-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('place in-front-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('place behind the box'),
            RC_PLACE['PL_' + rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('place on-top-of the box'),
            RC_PLACE['PL_' + rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('place near the box'),
            RC_PLACE['PL_' + rc_key_short + '_NEAR'])


class FullObjectPossibleRobotPreferredMismatch(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [WorldObject(O_FULL_REACHABLE)]
        # NOTE: Only 'update objects' called; this ensures we must
        # update the robot in each test method before the parser will
        # run (as we want both to be specified). We could call 'set
        # world' instead but this ensures we write the tests correctly.
        self.frontend.update_objects(world_objects=objs)

    def test_left_preferred_right_requested(self):
        self.frontend.update_robot(Robot(R_LEFT_PREF))
        self._check_side_requested('right_hand')

    def test_right_preferred_left_requested(self):
        self.frontend.update_robot(Robot(R_RIGHT_PREF))
        self._check_side_requested('left_hand')

    def _check_side_requested(self, side_rc):
        '''
        Args:
            side_rc: 'right_hand' or 'left_hand'
        '''
        # Make request str
        side_req = 'right-hand' if side_rc == 'right_hand' else 'left-hand'

        # calculate keys for RC maps
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'

        # open / close
        self.assertEqual(
            self.frontend.parse('open ' + side_req),
            RC_OPEN_CLOSE['OPEN' + rc_key_long])
        self.assertEqual(
            self.frontend.parse('close ' + side_req),
            RC_OPEN_CLOSE['CLOSE' + rc_key_long])

        # moveabs
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' up'),
            RC_MOVEABS[rc_key_short + '_UP'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' down'),
            RC_MOVEABS[rc_key_short + '_DOWN'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' left'),
            RC_MOVEABS[rc_key_short + '_LEFT'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' right'),
            RC_MOVEABS[rc_key_short + '_RIGHT'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' forward'),
            RC_MOVEABS[rc_key_short + '_FORWARD'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' backwards'),
            RC_MOVEABS[rc_key_short + '_BACKWARD'])

        # moverel
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' above the box'),
            RC_MOVEREL[rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' next-to the box'),
            RC_MOVEREL[rc_key_short + '_NEXTTO'])

        self.assertEqual(
            self.frontend.parse(
                'move ' + side_req + ' to-the-left-of the box'),
            RC_MOVEREL[rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse(
                'move ' + side_req + ' to-the-right-of the box'),
            RC_MOVEREL[rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' in-front-of the box'),
            RC_MOVEREL[rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' behind the box'),
            RC_MOVEREL[rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' on-top-of the box'),
            RC_MOVEREL[rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' near the box'),
            RC_MOVEREL[rc_key_short + '_NEAR'])

        # movereldir
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' away-from the box'),
            RC_MOVEREL_DIR[rc_key_short + '_AWAY'])
        self.assertEqual(
            self.frontend.parse('move ' + side_req + ' towards the box'),
            RC_MOVEREL_DIR[rc_key_short + '_TOWARDS'])

        # pickup
        self.assertEqual(
            self.frontend.parse('pick-up with ' + side_req),
            RC_PICKUP[rc_key_short])

        # place
        self.assertEqual(
            self.frontend.parse('place above the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.frontend.parse('place next-to the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO'])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-left-of the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_LEFTOF'])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-right-of the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_RIGHTOF'])
        self.assertEqual(
            self.frontend.parse('place in-front-of the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_FRONTOF'])
        self.assertEqual(
            self.frontend.parse('place behind the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_BEHIND'])
        self.assertEqual(
            self.frontend.parse('place on-top-of the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_TOPOF'])
        self.assertEqual(
            self.frontend.parse('place near the box with ' + side_req),
            RC_PLACE['PL_' + rc_key_short + '_NEAR'])


class FullMultiObjectsSimple(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [
            WorldObject(O_FULL_REACHABLE),  # obj0
            WorldObject(O_FULL_REACHABLE_SECOND),  # obj1
        ]
        self.frontend.set_world(world_objects=objs)

    def test_select_obj0(self):
        hands = ['right_hand', 'left_hand']
        descs = ['red box', 'red thing', 'red object', 'box', 'crimson box']
        for hand in hands:
            for desc in descs:
                self._check_object(hand, 'obj0', desc)

    def test_select_obj1(self):
        hands = ['right_hand', 'left_hand']
        descs = ['blue cup', 'blue thing', 'blue object', 'cup']
        for hand in hands:
            for desc in descs:
                self._check_object(hand, 'obj1', desc)

    def _check_object(self, side_rc, obj_str, desc):
        '''
        Args:
            side_rc (str): 'right_hand' or 'left_hand'
            obj_str (str): 'obj0' or 'obj1'
            desc (str): Descriptor for the object

        '''
        # calculate keys for RC maps
        hand_str = 'right-hand' if side_rc == 'right_hand' else 'left-hand'
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'
        objkey = '2' if obj_str == 'obj1' else ''

        # moverel
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' above the ' + desc),
            RC_MOVEREL[rc_key_short + '_ABOVE' + objkey])
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' next-to the ' + desc),
            RC_MOVEREL[rc_key_short + '_NEXTTO' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move ' + hand_str + ' to-the-left-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_LEFTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move ' + hand_str + ' to-the-right-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_RIGHTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move ' + hand_str + ' in-front-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_FRONTOF' + objkey])
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' behind the ' + desc),
            RC_MOVEREL[rc_key_short + '_BEHIND' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move ' + hand_str + ' on-top-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_TOPOF' + objkey])
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' near the ' + desc),
            RC_MOVEREL[rc_key_short + '_NEAR' + objkey])

        # movereldir
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' away-from the ' + desc),
            RC_MOVEREL_DIR[rc_key_short + '_AWAY' + objkey])
        self.assertEqual(
            self.frontend.parse('move ' + hand_str + ' towards the ' + desc),
            RC_MOVEREL_DIR[rc_key_short + '_TOWARDS' + objkey])

        # pickup
        self.assertEqual(
            self.frontend.parse('pick-up the ' + desc + ' with ' + hand_str),
            RC_PICKUP[rc_key_short + objkey])

        # place
        self.assertEqual(
            self.frontend.parse(
                'place above the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_ABOVE' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place next-to the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-left-of the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_LEFTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-right-of the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_RIGHTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place in-front-of the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_FRONTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place behind the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_BEHIND' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place on-top-of the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_TOPOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place near the ' + desc + ' with ' + hand_str),
            RC_PLACE['PL_' + rc_key_short + '_NEAR' + objkey])


class FullMultiObjectsPickHand(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [
            WorldObject(O_RIGHT_POSSIBLE),  # obj0
            WorldObject(O_LEFT_POSSIBLE_SECOND),  # obj1
        ]
        self.frontend.set_world(world_objects=objs)

    def test_pick_right(self):
        descs = [
            'smallest red box',
            'right-most red box',
            'smallest thing',
            'right-most thing'
        ]
        for desc in descs:
            self._check_object('right_hand', 'obj0', desc)

    def test_pick_left(self):
        descs = [
            'biggest box',
            'left-most box',
            'biggest thing',
            'left-most thing'
        ]
        for desc in descs:
            self._check_object('left_hand', 'obj1', desc)

    def _check_object(self, side_rc, obj_str, desc):
        '''
        Args:
            side_rc (str): 'right_hand' or 'left_hand'
            obj_str (str): 'obj0' or 'obj1'
            desc (str): Descriptor for the object

        '''
        # calculate keys for RC maps
        rc_key_long = 'RIGHT' if side_rc == 'right_hand' else 'LEFT'
        rc_key_short = 'RH' if side_rc == 'right_hand' else 'LH'
        objkey = '2' if obj_str == 'obj1' else ''

        # moverel
        self.assertEqual(
            self.frontend.parse('move above the ' + desc),
            RC_MOVEREL[rc_key_short + '_ABOVE' + objkey])
        self.assertEqual(
            self.frontend.parse('move next-to the ' + desc),
            RC_MOVEREL[rc_key_short + '_NEXTTO' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move to-the-left-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_LEFTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'move to-the-right-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_RIGHTOF' + objkey])
        self.assertEqual(
            self.frontend.parse('move in-front-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_FRONTOF' + objkey])
        self.assertEqual(
            self.frontend.parse('move behind the ' + desc),
            RC_MOVEREL[rc_key_short + '_BEHIND' + objkey])
        self.assertEqual(
            self.frontend.parse('move on-top-of the ' + desc),
            RC_MOVEREL[rc_key_short + '_TOPOF' + objkey])
        self.assertEqual(
            self.frontend.parse('move near the ' + desc),
            RC_MOVEREL[rc_key_short + '_NEAR' + objkey])

        # movereldir
        self.assertEqual(
            self.frontend.parse('move away-from the ' + desc),
            RC_MOVEREL_DIR[rc_key_short + '_AWAY' + objkey])
        self.assertEqual(
            self.frontend.parse('move towards the ' + desc),
            RC_MOVEREL_DIR[rc_key_short + '_TOWARDS' + objkey])

        # pickup
        self.assertEqual(
            self.frontend.parse('pick-up the ' + desc),
            RC_PICKUP[rc_key_short + objkey])

        # place
        self.assertEqual(
            self.frontend.parse('place above the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_ABOVE' + objkey])
        self.assertEqual(
            self.frontend.parse('place next-to the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-left-of the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_LEFTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place to-the-right-of the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_RIGHTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place in-front-of the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_FRONTOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place behind the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_BEHIND' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place on-top-of the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_TOPOF' + objkey])
        self.assertEqual(
            self.frontend.parse(
                'place near the ' + desc),
            RC_PLACE['PL_' + rc_key_short + '_NEAR' + objkey])


class FullImpossibleRobotCommands(unittest.TestCase):
    '''
    Cases where you ask it to move in some way and neither hand can. The
    idea is the language should be 'so heavily weighted' that either the
    robot, or our model, should accept the command as requested and just
    say it can't do it. (I think this is a job for the robot).
    '''

    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()

    def test_impossible_openclose(self):
        robot = Robot({
            'gripper_states': ['closed_empty', 'open'],
            'last_cmd_side': 'right_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('open left-hand'),
            RC_OPEN_CLOSE['OPENLEFT']
        )
        self.assertEqual(
            self.frontend.parse('close right-hand'),
            RC_OPEN_CLOSE['CLOSERIGHT']
        )

        robot = Robot({
            'gripper_states': ['open', 'has_obj'],
            'last_cmd_side': 'left_hand',
        })
        self.frontend.set_world(robot=robot)
        self.assertEqual(
            self.frontend.parse('release right-hand'),
            RC_OPEN_CLOSE['OPENRIGHT']
        )
        self.assertEqual(
            self.frontend.parse('close left-hand'),
            RC_OPEN_CLOSE['CLOSELEFT']
        )

    def test_moveabs(self):
        self.frontend.set_world(robot=Robot(R_NEITHER_POSSIBLE))
        for cmd in S_MOVEABS.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEABS[cmd]), RC_MOVEABS[cmd])


class FullImpossibleObjCommands(unittest.TestCase):
    '''
    Cases where you ask it to pick up an object and neither hand can.
    The idea is the language should be 'so heavily weighted' that either
    the robot, or our model, should accept the command as requested and
    just say it can't do it. (I think this is a job for the robot).
    '''

    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.frontend = Frontend()
        objs = [
            WorldObject(O_IMPOSSIBLE),  # obj0
        ]
        self.frontend.set_world(world_objects=objs)

    def test_moverel(self):
        for cmd in S_MOVEREL.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEREL[cmd]), RC_MOVEREL[cmd])

    def test_movereldir(self):
        for cmd in S_MOVEREL_DIR.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_MOVEREL_DIR[cmd]), RC_MOVEREL_DIR[cmd])

    def test_pickup(self):
        for cmd in S_PICKUP.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_PICKUP[cmd]), RC_PICKUP[cmd])

    def test_place(self):
        for cmd in S_PLACE.iterkeys():
            self.assertEqual(self.frontend.parse(
                S_PLACE[cmd]), RC_PLACE[cmd])


# TODO: This is where we really test the tuning of the system. We need
#       to have the weights such that impossible AND unpreferred
#       commands are still returned if the person said them. This
#       reduces the inference to essentially filling in the gaps.
# class FullImpossibleUnpreferredCommands(unittest.TestCase):


# ######################################################################
# Main (execution starts here)
# ######################################################################

if __name__ == '__main__':
    unittest.main()
