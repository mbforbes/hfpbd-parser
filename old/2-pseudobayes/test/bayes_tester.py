#!/usr/bin/env python

'''Tests for hfpbd-parser.'''

__author__ = 'mbforbes'


# Imports
########################################################################

# Builtins
import unittest
import yaml

# Local
import bayes_parser


# Settings
########################################################################

BASIC_CORPUS_FILE = 'test/test_corpus_basic.yml'


# Classes
########################################################################

class TestBayesParser(unittest.TestCase):
    '''Tests for hfpbd Bayes parser.'''

    def setUp(self):
        self.p = bayes_parser.BayesParser()


# Generate tests at import time
########################################################################

# Function for making tests
def create_test(pair):
    # The test functions that will be run.
    def test_expected(self):
        self.assertEqual(
            self.p.parse_return_command(pair[0]),
            pair[1]
        )
    return test_expected

# Test group 1: test corpus basic
#
# Hits the parser with utterances and checks that the robot
# commands returned are as expected.
#
# Loads test from test file, so this isn't actually 'one test',
# but probably dozens.
#
# Note that this doesn't test additional inference methods (robot
# state, world state, command history). It also doesn't test the
# speech responses the robot gives. It also doesn't test the
# completeness of the grammar (i.e. command generation).
corpus_raw = yaml.load(open(BASIC_CORPUS_FILE))
for test_group_name, tests in corpus_raw.iteritems():
    for idx, test in enumerate(tests):
        test_method = create_test(test)
        test_method.__name__ = 'test_%s_%d' % (test_group_name, idx + 1)
        setattr(TestBayesParser, test_method.__name__, test_method)


# If run as main
########################################################################

if __name__ == '__main__':
    unittest.main()
