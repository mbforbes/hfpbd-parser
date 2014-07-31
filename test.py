'''Tests for hybridbayes hfpbd-parser.

Current test categories include:
    - Matcher: matcher tests (phrase matching)
    - Full: end-to-end tests (of entire system)
'''

# ######################################################################
# Imports
# ######################################################################

# Builtins
import getpass
import unittest

# Local
from hybridbayes import Parser, WorldObject, Robot, RobotCommand, Info, Debug
from matchers import NotSideMatcher


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
    'LH_UP': 'move left hand up',
    'LH_DOWN': 'move left hand down',
    'LH_LEFT': 'move left hand left',
    'LH_RIGHT': 'move left hand right',
    'RH_UP': 'move right hand up',
    'RH_DOWN': 'move right hand down',
    'RH_LEFT': 'move right hand left',
    'RH_RIGHT': 'move right hand right',
}
S_MOVEREL = {
    'RH_ABOVE': 'move right hand above the red box',
    'RH_NEXTTO': 'move right hand next to the red box',
    'LH_ABOVE': 'move left hand above the red box',
    'LH_NEXTTO': 'move left hand next to the red box',

}
S_PICKUP = {
    'RH': 'pick up the red box with your right hand',
    'LH': 'pick up the red box with your left hand',
}
S_PLACE = {
    'PL_RH_ABOVE': 'place above the red box with your right hand',
    'PL_RH_NEXTTO': 'place next to the red box with your right hand',
    'PL_LH_ABOVE': 'place above the red box with your left hand',
    'PL_LH_NEXTTO': 'place next to the red box with your left hand',
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
RC_MOVEABS = {
    'LH_UP': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'up']),
    'LH_DOWN': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'down']),
    'LH_LEFT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_left']),
    'LH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['left_hand', 'to_right']),
    'RH_UP': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'up']),
    'RH_DOWN': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'down']),
    'RH_LEFT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_left']),
    'RH_RIGHT': RobotCommand.from_strs(
        'move_abs', ['right_hand', 'to_right']),
}
RC_MOVEREL = {
    'RH_ABOVE': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'above', 'obj0']),
    'RH_NEXTTO': RobotCommand.from_strs(
        'move_rel', ['right_hand', 'next_to', 'obj0']),
    'LH_ABOVE': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'above', 'obj0']),
    'LH_NEXTTO': RobotCommand.from_strs(
        'move_rel', ['left_hand', 'next_to', 'obj0']),

}
RC_PICKUP = {
    'RH': RobotCommand.from_strs(
        'pick_up', ['obj0', 'right_hand']),
    'LH': RobotCommand.from_strs(
        'pick_up', ['obj0', 'left_hand']),
}
RC_PLACE = {
    'PL_RH_ABOVE': RobotCommand.from_strs(
        'place', ['above', 'obj0', 'right_hand']),
    'PL_RH_NEXTTO': RobotCommand.from_strs(
        'place', ['next_to', 'obj0', 'right_hand']),
    'PL_LH_ABOVE': RobotCommand.from_strs(
        'place', ['above', 'obj0', 'left_hand']),
    'PL_LH_NEXTTO': RobotCommand.from_strs(
        'place', ['next_to', 'obj0', 'left_hand']),
}
RC_LOOKAT = {
    'LOOKAT': RobotCommand.from_strs(
        'look_at', ['obj0']),
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

# Only the one side is 'possible', where applicable.
O_RIGHT_POSSIBLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [True, False],
    'is_above_reachable': [True, False],
    'is_nextto_reachable': [True, False],
    # E.g. red, blue, green, unknown
    'color': 'red',
    'has_distinct_color': True,
    # E.g. cup, box, unknown
    'type': 'box',
    'has_distinct_type': True,
}
O_LEFT_POSSIBLE = {
    'name': 'obj0',
    # Relation to arms.
    'is_pickupable': [False, True],
    'is_above_reachable': [False, True],
    'is_nextto_reachable': [False, True],
    # E.g. red, blue, green, unknown
    'color': 'red',
    'has_distinct_color': True,
    # E.g. cup, box, unknown
    'type': 'box',
    'has_distinct_type': True,
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
}

R_ONLY_LEFT_POSSIBLE = {
    'can_move_up': [False, True],
    'can_move_down': [False, True],
    'can_move_toleft': [False, True],
    'can_move_toright': [False, True],
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
        return getpass.getuser() == 'travis'


class TestNotSideMatcher(unittest.TestCase):
    def test_unmatches(self):
        sides = ['right', 'left']
        for i, side in enumerate(sides):
            self.assertFalse(NotSideMatcher.match(
                side, side + ' hand'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' arm'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' gripper'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' arm ' + side + ' hand'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' arm ' + side + ' gripper'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' hand ' + side + ' arm'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' hand ' + side + ' gripper'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' gripper ' + side + ' hand'))
            self.assertFalse(NotSideMatcher.match(
                side, side + ' gripper ' + side + ' arm'))

    def test_matches(self):
        sides = ['right', 'left']
        for i, side in enumerate(sides):
            self.assertTrue(NotSideMatcher.match(
                side, side + ' ' + side + ' hand'))
            self.assertTrue(NotSideMatcher.match(
                side, side + ' ' + side + ' gripper'))
            self.assertTrue(NotSideMatcher.match(
                side, side + ' ' + side + ' arm'))
            self.assertTrue(NotSideMatcher.match(
                side, side + ' hand ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, side + ' gripper ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, side + ' arm ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move right hand ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move left hand ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move right arm ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move left arm ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move right gripper ' + side))
            self.assertTrue(NotSideMatcher.match(
                side, 'move left gripper ' + side))


class FullNoContext(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        # TODO(mbforbes): Remove this once we know it works.
        print 'Running in debug?', TestUtil.on_travis()

        self.parser = Parser(debug=TestUtil.on_travis())
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
        self.parser = Parser(debug=TestUtil.on_travis())
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

    def test_moverel(self):
        for cmd in S_MOVEREL.iterkeys():
            self.assertEqual(self.parser.parse(
                S_MOVEREL[cmd])[0], RC_MOVEREL[cmd])

    def test_pickup(self):
        for cmd in S_PICKUP.iterkeys():
            self.assertEqual(self.parser.parse(
                S_PICKUP[cmd])[0], RC_PICKUP[cmd])

    def test_place(self):
        for cmd in S_PLACE.iterkeys():
            self.assertEqual(self.parser.parse(
                S_PLACE[cmd])[0], RC_PLACE[cmd])

    def test_lookat(self):
        for cmd in S_LOOKAT.iterkeys():
            self.assertEqual(self.parser.parse(
                S_LOOKAT[cmd])[0], RC_LOOKAT[cmd])

    def test_synonyms(self):
        self.assertEqual(
            self.parser.parse('open left gripper')[0],
            RC_OPEN_CLOSE['OPENLEFT'])
        self.assertEqual(
            self.parser.parse('close right gripper')[0],
            RC_OPEN_CLOSE['CLOSERIGHT'])
        self.assertEqual(
            self.parser.parse('move left arm higher')[0],
            RC_MOVEABS['LH_UP'])
        self.assertEqual(
            self.parser.parse('move right gripper to the left')[0],
            RC_MOVEABS['RH_LEFT'])
        self.assertEqual(
            self.parser.parse('pick the red block up left hand')[0],
            RC_PICKUP['LH'])
        self.assertEqual(
            self.parser.parse('pick the red block up right arm')[0],
            RC_PICKUP['RH'])

    def test_missing_words(self):
        self.assertEqual(
            self.parser.parse('lower left gripper')[0],
            RC_MOVEABS['LH_DOWN'])
        self.assertEqual(
            self.parser.parse('raise right gripper')[0],
            RC_MOVEABS['RH_UP'])

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
                    actual_rc = self.parser.parse(query)[0]
                    self.assertEqual(expected_rc, actual_rc)

    def test_obj_desc(self):
        self.assertEqual(
            self.parser.parse('pick up that thing left hand')[0],
            RC_PICKUP['LH'])
        self.assertEqual(
            self.parser.parse('pick up the box right hand')[0],
            RC_PICKUP['RH'])
        self.assertEqual(
            self.parser.parse('pick up the red thing right hand')[0],
            RC_PICKUP['RH'])


class FullOneObjRobotSidePref(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.parser = Parser(debug=TestUtil.on_travis())
        objs = [WorldObject(O_FULL_REACHABLE)]
        # NOTE: Only 'update objects' called; this ensures we must
        # update the robot in each test method before the parser will
        # run (as we want both to be specified). We could call 'set
        # world' instead but this ensures we write the tests correctly.
        self.parser.update_objects(world_objects=objs)

    def test_right_preferred(self):
        self.parser.update_robot(Robot(R_RIGHT_PREF))
        self._check_side_preferred('right_hand')

    def test_left_preferred(self):
        self.parser.update_robot(Robot(R_LEFT_PREF))
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
            self.parser.parse('open')[0],
            RC_OPEN_CLOSE['OPEN' + rc_key_long])
        self.assertEqual(
            self.parser.parse('close')[0],
            RC_OPEN_CLOSE['CLOSE' + rc_key_long])

        # moveabs
        self.assertEqual(
            self.parser.parse('move up')[0],
            RC_MOVEABS[rc_key_short + '_UP'])
        self.assertEqual(
            self.parser.parse('move down')[0],
            RC_MOVEABS[rc_key_short + '_DOWN'])
        self.assertEqual(
            self.parser.parse('move left')[0],
            RC_MOVEABS[rc_key_short + '_LEFT'])
        self.assertEqual(
            self.parser.parse('move right')[0],
            RC_MOVEABS[rc_key_short + '_RIGHT'])

        # moverel
        self.assertEqual(
            self.parser.parse('move above the box')[0],
            RC_MOVEREL[rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.parser.parse('move next to the box')[0],
            RC_MOVEREL[rc_key_short + '_NEXTTO'])

        # pickup, place
        self.assertEqual(
            self.parser.parse('pick up')[0],
            RC_PICKUP[rc_key_short])
        self.assertEqual(
            self.parser.parse('place above the box')[0],
            RC_PLACE['PL_' + rc_key_short + '_ABOVE'])
        self.assertEqual(
            self.parser.parse('place next to the box')[0],
            RC_PLACE['PL_' + rc_key_short + '_NEXTTO'])


class FullRobotOneSidePossible(unittest.TestCase):
    def setUp(self):
        Info.printing = False
        Debug.printing = False
        self.parser = Parser(debug=TestUtil.on_travis())

    def test_only_right_possible(self):
        # Trying with no objects; focus is on ROBOT state.
        robot = Robot(R_ONLY_RIGHT_POSSIBLE)
        self.parser.set_world(robot=robot)
        self._check_side_possible('right_hand')

    def test_only_left_possible(self):
        # Trying with no objects; focus is on ROBOT state.
        robot = Robot(R_ONLY_LEFT_POSSIBLE)
        self.parser.set_world(robot=robot)
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
            self.parser.parse('move up')[0],
            RC_MOVEABS[rc_key_short + '_UP'])
        self.assertEqual(
            self.parser.parse('move down')[0],
            RC_MOVEABS[rc_key_short + '_DOWN'])
        self.assertEqual(
            self.parser.parse('move left')[0],
            RC_MOVEABS[rc_key_short + '_LEFT'])
        self.assertEqual(
            self.parser.parse('move right')[0],
            RC_MOVEABS[rc_key_short + '_RIGHT'])

# TODO
# class FullObjectOneSidePossible(unittest.TestCase):

# TODO
# class FullObjectPossibleRobotPreferredMismatch(unittest.TestCase):

# TODO
# class FullObjectDescripers

# TODO
# class FullMultiObjects

# TODO: Cases where you ask it to pick up an object and neither hand
#       can. The idea is the language should be 'so heavily weighted'
#       that either the robot, or our model, should accept the command
#       as requested and just say it can't do it. (I think this is a job
#       for the robot).
# class FullImpossibleCommands


# ######################################################################
# Main (execution starts here)
# ######################################################################

if __name__ == '__main__':
    unittest.main()
