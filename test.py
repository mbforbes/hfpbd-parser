'''Tests for hybridbayes hfpbd-parser.

Current test categories include:
    - Matcher: matcher tests (phrase matching)
    - Full: end-to-end tests (of entire system)
'''

# ######################################################################
# Imports
# ######################################################################

# Builtins
import unittest

# Local
from hybridbayes import Parser, WorldObject, RobotCommand, Info, Debug
# TODO markers


# ######################################################################
# Constants (will have to keep sync'd with grammar file)
# ######################################################################

# Strings (S)
S_OPEN_CLOSE = {
    'OPENRIGHT': 'open right hand',
    'OPENLEFT': 'open left hand',
    'CLOSERIGHT': 'close right hand',
    'CLOSELEFT': 'close left hand',
}
S_MOVEABS = {
    'MOVEABS_LH_UP': 'move left hand up',
    'MOVEABS_LH_DOWN': 'move left hand down',
    'MOVEABS_LH_LEFT': 'move left hand left',
    'MOVEABS_LH_RIGHT': 'move left hand right',
    'MOVEABS_RH_UP': 'move right hand up',
    'MOVEABS_RH_DOWN': 'move right hand down',
    'MOVEABS_RH_LEFT': 'move right hand left',
    'MOVEABS_RH_RIGHT': 'move right hand right',
}

# RobotCommands (RC)
RC_OPEN_CLOSE = {
    'OPENRIGHT': RobotCommand.from_strs('open', ['right_hand']),
    'OPENLEFT': RobotCommand.from_strs('open', ['left_hand']),
    'CLOSERIGHT': RobotCommand.from_strs('close', ['right_hand']),
    'CLOSELEFT': RobotCommand.from_strs('close', ['left_hand']),
}
RC_MOVEABS = {
    'MOVEABS_LH_UP': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'up']),
    'MOVEABS_LH_DOWN': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'down']),
    'MOVEABS_LH_LEFT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_left']),
    'MOVEABS_LH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_right']),
    'MOVEABS_RH_UP': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'up']),
    'MOVEABS_RH_DOWN': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'down']),
    'MOVEABS_RH_LEFT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_left']),
    'MOVEABS_RH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_right']),
}

# Objects (O)
O_FULL_REACHABLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [True, True],
    'is_above_reachable': [True, True],
    'is_nextto_reachable': [True, True],
    # Relation to other objects. These should be more general.
    'is_leftmost': False,
    'is_righttmost': False,
    'is_biggest': False,
    'is_smallest': False,
    # E.g. red, blue, green, unknown
    'color': 'red',
    'has_distinct_color': True,
    # E.g. cup, box, unknown
    'type': 'box',
    'has_distinct_type': True,
}

# ######################################################################
# Classes (test categories)
# ######################################################################

class FullNoContext(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.parser = Parser()
        self.parser.set_world()

    def test_openclose(self):
        for cmd in S_OPEN_CLOSE.iterkeys():
            self.assertEqual(self.parser.parse(
                S_OPEN_CLOSE[cmd])[0], RC_OPEN_CLOSE[cmd])

    def test_moveabs(self):
        for cmd in S_MOVEABS.iterkeys():
            self.assertEqual(self.parser.parse(
                S_MOVEABS[cmd])[0], RC_MOVEABS[cmd])

class FullOneObjNoRobot(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.parser = Parser()
        objs = [WorldObject(O_FULL_REACHABLE)]
        self.parser.set_world(world_objects=objs)

    def test_openclose(self):
        for cmd in S_OPEN_CLOSE.iterkeys():
            self.assertEqual(self.parser.parse(
                S_OPEN_CLOSE[cmd])[0], RC_OPEN_CLOSE[cmd])

    def test_moveabs(self):
        for cmd in S_MOVEABS.iterkeys():
            self.assertEqual(self.parser.parse(
                S_MOVEABS[cmd])[0], RC_MOVEABS[cmd])

# ######################################################################
# Main (execution starts here)
# ######################################################################

if __name__ == '__main__':
    unittest.main()
