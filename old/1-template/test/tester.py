#!/usr/bin/env python

'''Tests for hfpbd-parser.'''

# Builtins
import unittest

# Local
import parser


class TestParser(unittest.TestCase):
    '''Tests for hfpbd-parser.'''

    def setUp(self):
        self.p = parser.Parser()

    def test_ll_wrist_basic(self):
        # Clockwise
        self.assertEqual(
            self.p.parse('Rotate right wrist clockwise'),
            parser.Command.ROTATE_RIGHT_WRIST_CW
        )
        self.assertEqual(
            self.p.parse('Rotate left wrist clockwise'),
            parser.Command.ROTATE_LEFT_WRIST_CW
        )
        # Counter-clockwise
        self.assertEqual(
            self.p.parse('Rotate right wrist counterclockwise'),
            parser.Command.ROTATE_RIGHT_WRIST_CCW
        )
        self.assertEqual(
            self.p.parse('Rotate left wrist counterclockwise'),
            parser.Command.ROTATE_LEFT_WRIST_CCW
        )

    def test_ll_wrist_infer(self):
        # Clockwise
        self.assertEqual(
            self.p.parse('Rotate right wrist clockwise'),
            parser.Command.ROTATE_RIGHT_WRIST_CW
        )
