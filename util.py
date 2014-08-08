'''Helper classes and functions.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################


# ######################################################################
# Constants
# ######################################################################

# Logging
ERROR_PRINTING_DEFAULT = True
INFO_PRINTING_DEFAULT = True
DEBUG_PRINTING_DEFAULT = True

# Numbers
FLOAT_COMPARE_EPSILON = 0.001
LENGTH_EXP = 10.0  # Polynomial degree for weight by length (x^this).


# ######################################################################
# Classes
# ######################################################################

class Logger(object):
    '''
    Handles logging. Optionally saves output in buffer for displaying.

    Subclass to set logging prefix and individual logging levels.
    '''

    buffer_printing = False
    printing = False
    prefix = '[IMPLEMENT ME]'
    print_buffer = []

    @classmethod
    def p(cls, obj):
        cls.pl(0, obj)

    @classmethod
    def pl(cls, level, obj):
        '''
        Args:
            level (int): how many tabs (one space if 0)
            obj (Object): what to print
        '''
        string = str(obj)
        if cls.printing:
            # tab = '\t'  # use for 'normal' tabs
            tab = '  '  # use for 'space' tabs (adjust as needed)
            indent = ' ' if level == 0 else tab * (level + 1)
            output = ''.join([cls.prefix, indent, string])
            if Logger.buffer_printing:
                # Save for later
                Logger.print_buffer += [output]
            else:
                print output

    @staticmethod
    def get_buffer():
        '''
        Empties and returns buffer.

        Returns:
            str
        '''
        retstr = '\n'.join(Logger.print_buffer)
        Logger.print_buffer = []
        return retstr


# Error
class Error(Logger):
    printing = ERROR_PRINTING_DEFAULT
    prefix = '[ERROR]'


# Debugging
class Info(Logger):
    printing = INFO_PRINTING_DEFAULT
    prefix = '[INFO]'


# Debugging
class Debug(Logger):
    printing = DEBUG_PRINTING_DEFAULT
    prefix = '[DEBUG]'


class Numbers:
    '''
    Misc. helper functionality related to numbers.
    '''

    @staticmethod
    def are_floats_close(a, b, epsilon=FLOAT_COMPARE_EPSILON):
        '''Checks whether two floats are within epsilon of each other.

        Args:
            a (float): One number.
            b (float): The other number.
            epsilon (float): Acceptable wiggle room (+/-) between a and
                b.

        Returns:
            bool: Whether a and b are within epsilon of each other.
        '''
        # We try to do this in an overflow-friendly way, though it
        # probably isn't a big deal with our use cases and python.
        return a - epsilon <= b if a > b else b - epsilon <= a

    @staticmethod
    def boost_list(nums, min_score=0.0):
        '''
        Boosts a list of floats so they are all at least min_score.

        Args:
            nums ([float]):
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to 0.0.
        Returns:
            [float]
        '''
        min_ = min(nums)
        boost = min_score - min_ if min_score > min_ else 0.0
        return [n + boost for n in nums]

    @staticmethod
    def normalize_list(nums, min_score=0.0, scale=1.0):
        '''
        Normalizes list of floats to sum to scale (a valid probability
        distribution if scale = 1.0).

        Args:
            nums ([float]):
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to 0.0.
            scale (float, optional): What nums will sum to. Defaults to
                1.0.

        Returns:
            [float]
        '''
        # First, boost all to some minimum value.
        nums = Numbers.boost_list(nums, min_score)

        # Next, do the normalization.
        sum_ = sum(nums)
        # Avoid divide by zero.
        if sum_ == 0.0:
            return [0.0] * len(nums)
        # Else actually normalize.
        return [(n / sum_) * scale for n in nums]

    @staticmethod
    def boost(objs, attr='score', min_score=0.0):
        '''
        Boosts a list of objects with a attr attribute (that is a float)
        so they are all >= min_score.

        Args:
            objs ([Object]): List of Objects
            attr (str, optional): The name of the attribute to extract
                from objects. Defaults to 'score'.
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to 0.0
        '''
        nums = [getattr(obj, attr) for obj in objs]
        nums = Numbers.boost_list(nums, min_score)
        for i in range(len(objs)):
            setattr(objs[i], attr, nums[i])

    @staticmethod
    def make_prob(objs, attr='score'):
        '''
        Makes a list of objects with an attr attribute (that is a float)
        so they are all 0.0 < val < 1.0 by using the maximum as the 1.0
        value and doing exponential decay to 0.0.

        Args:
            objs ([Object]): List of Objects
            attr (str, optional): The name of the attribute to extract
                from objects. Defaults to 'score'.
        '''
        max_ = max([getattr(obj, attr) for obj in objs])
        if max_ == 0.0:
            for obj in objs:
                setattr(obj, attr, 1.0 / len(objs))
        else:
            for obj in objs:
                orig = getattr(obj, attr)
                new = (orig / max_)**LENGTH_EXP
                setattr(obj, attr, new)

    @staticmethod
    def normalize(objs, attr='score', min_score=0.0, scale=1.0):
        '''
        Normalizes list of objects with a attr attribute (that is a
        float) to a valid probability distribution.

        Args:
            objs ([Object]): List of Objects
            attr (str, optional): The name of the attribute to extract
                from objects. Defaults to 'score'.
            min_score (float, optional): The lowest score to boost
                objects to (all objects are boosted uniformly). Defaults
                to 0.0.
            scale (float, optional): What nums will sum to. Defaults to
                1.0.
        '''
        nums = [getattr(obj, attr) for obj in objs]
        nums = Numbers.normalize_list(nums, min_score, scale)
        for i in range(len(objs)):
            setattr(objs[i], attr, nums[i])


class Algo:
    '''
    Misc algorithms that occur multiple times.

    On second thought, maybe these functions should be in Option. Hmm.
    '''

    @staticmethod
    def gen_phrases(options):
        '''
        Generates an exhaustive list of lists of phrases from the passed
        list of Options.

        Args:
            options [Option]

        Returns:
            [[Phrase]]
        '''
        # The recursive method removes from the list, so we copy
        # references (shallow) first.
        opt_copy = options[:]
        return Algo._gen_phrases_recursive(opt_copy)

    @staticmethod
    def _gen_phrases_recursive(todo, results=[]):
        '''
        Recursively generates an exhaustive list of lists of phrases
        from the passed options in todo.

        Note that todo will be mutated (elements will be removed from it
        until it's empty) so this should be a shallow (ref) copy of any
        data structure that matters to the caller.

        Args:
            todo [Option]
            results ([[Phrase]])

        Returns:
            [[Phrase]]
        '''
        if len(todo) == 0:
            return results
        opt = todo.pop(0)
        next_phrases = opt.get_phrases()
        if results == []:
            new_results = next_phrases
        else:
            new_results = []
            for phrase_list in next_phrases:
                for r in results:
                    new_results += [r + phrase_list]
        return Algo._gen_phrases_recursive(todo, new_results)
