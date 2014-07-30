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

        all_clear = False
        for index in indexes:
            bw_found = False
            nextidx = index + len(words) + 1  # + 1 for space
            # If the next word is neither 'hand' nor 'arm', we've
            # matched NOT one of the side phrases.
            if nextidx < len(utterance):
                for bw in bad_follow_words:
                    possible_bw = utterance[nextidx:nextidx + len(bw)]
                    if possible_bw == bw:
                        bw_found = True
            # If this index has avoided all bad words, we're clear.
            if not bw_found:
                all_clear = True
                break
        return all_clear


class Matchers:
    # Indexes into classes
    MATCHERS = {
        'default': DefaultMatcher,
        'notside': NotSideMatcher
    }

# TODO(mbforbes): Put this in a test directory.
if __name__ == '__main__':
    sides = ['right', 'left']
    for i, side in enumerate(sides):
        assert not NotSideMatcher.match(side, side + ' hand')
        assert not NotSideMatcher.match(side, side + ' arm')
        assert not NotSideMatcher.match(side, side + ' arm ' + side + ' hand')
        assert not NotSideMatcher.match(side, side + ' hand ' + side + ' arm')

        assert NotSideMatcher.match(side, side + ' ' + side + ' hand')
        assert NotSideMatcher.match(side, side + ' hand ' + side)
        assert NotSideMatcher.match(side, side)
        assert NotSideMatcher.match(side, 'move right hand ' + side)
        assert NotSideMatcher.match(side, 'move left hand ' + side)
