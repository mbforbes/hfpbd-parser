'''Handles matching phrases in an utterance.'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import sys


########################################################################
# Classes
########################################################################

class MatchingStrategy:
    '''Interface for matching strategies.'''

    @staticmethod
    def match(words, utterance):
        Error.p("MatchingStrategy:match unimplemented as it's an interface.")
        sys.exit(1)


class DefaultMatcher:
    '''Default matching strategy.'''

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        return words in utterance


class NotSideMatcher:
    '''Matching strategy that avoids matching side utterances.'''

    @staticmethod
    def find_all(words, utterance):
        '''
        Finds all indexes of words in utterance.

        Args:
            words (str)
            utterance (str)

        Returns:
            [int]: indexes
        '''
        indexes = []
        start = 0
        res = utterance.find(words, start)
        while res > -1:
            indexes += [res]
            start = res + 1
            res = utterance.find(words, start)
        return indexes

    @staticmethod
    def match(words, utterance):
        '''Returns whether words match an utterance, avoiding side
        utterances.

        Args:
            words (str)
            utterance (str)

        Returns:
            bool
        '''
        bad_follow_words = ['hand', 'arm']
        # Get basic test out of the way.
        if words not in utterance:
            return False

        # At this point, the words are in the utterance. Check whether
        # we're really matching the phrase...
        indexes = NotSideMatcher.find_all(words, utterance)

        for index in indexes:
            nextidx = index + len(words) + 1  # + 1 for space
            # If the next word is neither 'hand' nor 'arm', we've
            # matched NOT one of the side phrases.
            if nextidx < len(utterance):
                for bw in bad_follow_words:
                    if utterance[nextidx:nextidx + len(bw)] != bw:
                        return True

        # We couldn't find any indexes that weren't just side indexes.
        # So there are no matches.
        return False


class Matchers:
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'notside': NotSideMatcher
    }
