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


LANG_MATCH_PARAM_SCORE = 1.0
LANG_GROUND_PARAM_SCORE = 0.0  # Normal words don't matter in grounding.

LANG_MATCH_VERB_SCORE = 5.0
LANG_GROUND_VERB_SCORE = 0.0  # Verbs don't matter in grounding.

LANG_MATCH_ADJ_SCORE = 1.0
LANG_GROUND_ADJ_SCORE = 0.4  # Adjectives matter more in grounding.

LANG_MATCH_NOUN_SCORE = 1.0
LANG_GROUND_NOUN_SCORE = 0.2  # Nouns matter less in grounding.


########################################################################
# Classes
########################################################################

class MatchingStrategy(object):
    '''Interface for matching strategies.'''

    # We don't put match_score or grond_score here as a matching
    # strategy must be a subclass.

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

    @classmethod
    def get_match_score(cls):
        '''
        Returns score that should be given for matching the phrase
        correctly in parsing.
        '''
        return cls.match_score

    @classmethod
    def get_ground_score(cls):
        '''
        Returns score that should be given for matching the phrase
        correctly in grounding.
        '''
        return cls.ground_score

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

    match_score = LANG_MATCH_PARAM_SCORE
    ground_score = LANG_GROUND_PARAM_SCORE


class VerbMatcher(MatchingStrategy):
    '''Matching strategy for verbs.'''

    match_score = LANG_MATCH_VERB_SCORE
    ground_score = LANG_GROUND_VERB_SCORE


class AdjectiveMatcher(MatchingStrategy):
    '''Matching strategy for object adjectives.'''

    match_score = LANG_MATCH_ADJ_SCORE
    ground_score = LANG_GROUND_ADJ_SCORE


class NounMatcher(MatchingStrategy):
    '''Matching strategy for object nouns.'''

    match_score = LANG_MATCH_NOUN_SCORE
    ground_score = LANG_GROUND_NOUN_SCORE


class Matchers(object):
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'verb': VerbMatcher,
        'adj': AdjectiveMatcher,
        'noun': NounMatcher,
    }
