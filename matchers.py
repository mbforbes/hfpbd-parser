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
        '''Returns whether words match an utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        return MatchingStrategy._words_in(words, utterance)

    @staticmethod
    def score():
        '''
        Returns score that should be given for matching the phrase
        correctly.
        '''
        Error.p("MatchingStrategy:score unimplemented as it's an interface.")
        sys.exit(1)

    @staticmethod
    def _words_in(words, utterance):
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


class DefaultMatcher(MatchingStrategy):
    '''Default matching strategy (for 'normal' parameters).'''

    @staticmethod
    def score():
        return MatchingStrategy.param_match


class VerbMatcher(MatchingStrategy):
    '''Matching strategy for verbs.'''

    @staticmethod
    def score():
        return MatchingStrategy.verb_match


class Matchers:
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'verb': VerbMatcher
    }
