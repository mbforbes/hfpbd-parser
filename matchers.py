'''Handles matching phrases in an utterance.'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import sys

# Local
from util import Debug


########################################################################
# Module-level constants
########################################################################

LANG_MATCH_VERB_SCORE = 5.0
LANG_MATCH_PARAM_SCORE = 1.0


########################################################################
# Classes
########################################################################

class MatchingStrategy:
    '''Interface for matching strategies.'''

    verb_match = LANG_MATCH_VERB_SCORE
    param_match = LANG_MATCH_PARAM_SCORE

    @staticmethod
    def match(words, utterance):
        Error.p("MatchingStrategy:match unimplemented as it's an interface.")
        sys.exit(1)

    @staticmethod
    def words_in(words, utterance):
        '''
        Returns whether all words in words are found somewhere in
        utterance.

        Args:
            words (str)
            utterance (str)
        '''
        pieces = utterance.split(' ')
        for word in words.split(' '):
            if word not in pieces:
                return False
        return True


class DefaultMatcher:
    '''Default matching strategy.'''

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            float: score
        '''
        return (
            MatchingStrategy.param_match
            if MatchingStrategy.words_in(words, utterance)
            else 0.0
        )


class VerbMatcher:
    '''Default matching strategy.'''

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance. words contains
        verbs.

        Args:
            words (str)
            utterance (str)

        Returns:
            float: score
        '''
        return (
            MatchingStrategy.verb_match
            if MatchingStrategy.words_in(words, utterance)
            else 0.0
        )


class Matchers:
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'verb': VerbMatcher
    }
