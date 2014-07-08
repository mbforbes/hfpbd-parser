'''Template-based Parser for hfpbd.'''

__author__ = 'mbforbes'


# Imports
########################################################################

from collections import defaultdict


# Constants
########################################################################

# Global options
DEBUG_PRINTING_DEFAULT = True


# Classes
########################################################################

# These are returned from the parser.

class Command:
    '''Commands returned from parser.'''

    # Rotate wrist
    ROTATE_LEFT_WRIST_CW = 'Rotate left wrist clockwise.'
    ROTATE_LEFT_WRIST_CCW = 'Rotate left wrist counterclockwise.'
    ROTATE_RIGHT_WRIST_CW = 'Rotate right wrist clockwise.'
    ROTATE_RIGHT_WRIST_CCW = 'Rotate right wrist counterclockwise.'

    # Open gripper
    OPEN_LEFT_GRIPPER = 'Open left gripper.'
    OPEN_RIGHT_GRIPPER = 'Open right gripper.'

    @staticmethod
    def build_rotate_wrist_command(options):
        '''Once ALL necessary options have been specified, call this to
        create a rotate wrist command.
        '''
        if Rotation.CLOCKWISE in options:
            if Side.LEFT in options:
                return Command.ROTATE_LEFT_WRIST_CW
            else:
                return Command.ROTATE_RIGHT_WRIST_CW
        else:
            # counter-clockwise rotation
            if Side.LEFT in options:
                return Command.ROTATE_LEFT_WRIST_CCW
            else:
                return Command.ROTATE_RIGHT_WRIST_CCW

    @staticmethod
    def build_open_gripper_command(options):
        '''Once ALL necessary options have been specified, call this to
        create an open gripper command.
        '''
        if Side.LEFT in options:
            return Command.OPEN_LEFT_GRIPPER
        else:
            return Command.OPEN_RIGHT_GRIPPER


class Clarification:
    '''Clarifications returned from parser.'''
    # General
    NO_KEYWORD = "I didn't hear any commands. Please rephrase."

    # Wrist rotation
    ROTATE_WRIST_WHICH_HOW = (
        'Please specify which wrist to rotate, and in which direction.')
    ROTATE_WRIST_WHICH = 'Please specifiy which wrist to rotate.'
    ROTATE_WRIST_CW_WHICH = 'Which wrist do I rotate clockwise?'
    ROTATE_WRIST_CCW_WHICH = 'Which wrist do I rotate counterclockwise?'
    ROTATE_LEFT_WRIST_DIR = 'Which direction do I rotate my left wrist?'
    ROTATE_RIGHT_WRIST_DIR = 'Which direction do I rotate my right wrist?'

    # Gripper opening
    OPEN_WHICH_GRIPPER = "Which gripper should I open?"

    # Methods to select these.

    @staticmethod
    def clarify_rotate_wrist_command(
            options, inferred, defaults, clarify_codes):
        '''Call this to construct a clarification request for a rotate
        wrist command.
        '''
        if len(clarify_codes) == 0:
            raise ValueError("Must provide at least one clarify code")
        if len(clarify_codes) == 2:
            # Need to clarify both
            return Clarification.ROTATE_WRIST_WHICH_HOW
        # One option to clarify
        if Clarifier.LEFT_OR_RIGHT in clarify_codes:
            # Clarifying left vs right.
            if Rotation.CLOCKWISE in options:
                return Clarification.ROTATE_WRIST_CW_WHICH
            elif Rotation.COUNTERCLOCKWISE in options:
                return Clarification.ROTATE_WRIST_CCW_WHICH
            else:
                # Counter- vs clockwise not specified; just ask for
                # side.
                return Clarification.ROTATE_WRIST_WHICH
        elif Clarifier.CW_OR_CCW in clarify_codes:
            # Clarifying clounter- vs clockwise.
            if Side.LEFT in options:
                return Clarification.ROTATE_LEFT_WRIST_DIR
            else:
                # Side.RIGHT in options
                return Clarification.ROTATE_RIGHT_WRIST_DIR
        else:
            raise ValueError(
                'Unexpected options to clarify_rotate_wrist_command: [' +
                ', '.join(options) + '] with codes: [' +
                ', '.join(clarify_codes) + '].')

    @staticmethod
    def clarify_open_gripper_command(
            options, inferred, defaults, clarify_codes):
        '''Call this to construct a clarification request for an open
        gripper command.
        '''
        return Clarification.OPEN_WHICH_GRIPPER


# These are used internally to avoid constants.

class Side:
    '''Class for determining side (right vs left).'''
    RIGHT = 'right'
    LEFT = 'left'


class Rotation:
    CLOCKWISE = 'clockwise'
    COUNTERCLOCKWISE = 'counterclockwise'


# This is the internal object used to represent robot state.

class RobotState:
    '''State of robot accounted for during parsing.'''
    pass


# Debugging
class Debug:
    printing = DEBUG_PRINTING_DEFAULT

    @staticmethod
    def p(string):
        Debug.pl(0, string)

    @staticmethod
    def pl(level, string):
        '''
        Args:
            level (int): how many tabs (one space if 0)
            string (str): string to print
        '''
        if Debug.printing:
            indent = ' ' if level == 0 else '\t' * (level + 1)
            print ''.join(['[DEBUG]', indent, string])

# Started attempt at general parser structure. Level/passing more
# complicated than anticipated; holding off for now.
# class KeywordFilter:
#     def __init__(self):
#         self.options = []

#     def add_option_group(self, keyword_filter):
#         self.options += [keyword_filter]


# This could be a useful general object for clarifying things.
class Clarifier:
    # Default value was used.
    DEFAULT = -2

    # Inferred from robot state.
    INFERRED = -1

    # Nothing to clarify.
    NONE = 0

    # Need to clarify left vs right.
    LEFT_OR_RIGHT = 1

    # Need to clarify clockwise vs counter-clockwise.
    CW_OR_CCW = 2

    def __init__(self, options, default=None, clarify_code=NONE):
        self.options = options
        self.default = default
        self.clarify_code = clarify_code

    def infer(self, robot_state):
        '''Try to infer from robot state.

        Returns: tuple of:
            bool: successfully inferred?,
            str|None: option if inferred or None
        '''
        # TODO: implement... will likely have to pass inference function
        # to use in constructor or something.
        return False, None

    def determine(self, utterance, robot_state):
        '''
        Returns: tuple of:
            bool: True if determined, inferred, or default
            str|None: option if determined or None,
            int: clarification state to add
        '''
        possibilities = self._get_possibilities(utterance)
        # Got multiple; try to resolve.
        if len(possibilities) > 1:
            return False, None, self.clarify_code
        # Got just one; use:
        if len(possibilities) == 1:
            return True, possibilities[0], Clarifier.NONE
        # Got no possibilities; see if we can infer from robot state.
        suc_infer, option = self.infer(robot_state)
        if suc_infer:
            return True, option, Clarifier.INFERRED
        # Still nothing; see if we can use default.
        if self.default is not None:
            return True, self.default, Clarifier.DEFAULT
        # Nothing said, couldn't use robot state, no deault: must
        # clarify.
        return False, None, self.clarify_code

    def _get_possibilities(self, utterance):
        '''Get possible options in an utterance, taking substrings into
        account.

        This means that if one option (shorter) is a substring of
        another (longer), and the longer one is seen, it is used wihtout
        conflict with the shorter one.

        This is only built with one level deep in mind.
        '''
        # Get all longer: [shorter1, shorter2, ...] substr pairs. This
        # tracks options substrings, where (shorter in longer) is True.
        substr_dict = defaultdict(list)
        for shorter in self.options:
            for longer in self.options:
                if shorter in longer and shorter != longer:
                    substr_dict[longer] += [shorter]
        # Find all possible options the naive way.
        possibilities = [o for o in self.options if o in utterance]
        Debug.p('Naive possibilities: ' + ', '.join(possibilities))
        # If a longer one has been seen, filter out shorter ones.
        for longer, shorters in substr_dict.iteritems():
            if longer in possibilities:
                # longer has been seen; remove all shorter
                for shorter in shorters:
                    if shorter in possibilities:
                        Debug.pl(
                            1, 'Removing ' + shorter + ' as it is in ' +
                            longer)
                        possibilities.remove(shorter)
        Debug.p('Pruned possibilities: ' + ', '.join(possibilities))
        return possibilities


class CommandParser:
    def __init__(
            self, keyword, clarifiers, command_builder, clarification_builder):
        self.keyword = keyword
        self.clarifiers = clarifiers
        self.command_builder = command_builder
        self.clarification_builder = clarification_builder

    def get_keyword(self):
        return self.keyword

    def parse(self, utterance, robot_state):
        '''This command parser has been assigned the utterance. Parse
        it.

        Returns: tuple of:
            str: Command.* to execute | Clarify* to reuest
            [int]: list of clarify codes to apply to parser
        '''
        # Need to apply all clarifiers.
        all_clarified = True
        clarify_codes = set()
        options, inferred, defaults = [], [], []
        for c in self.clarifiers:
            suc_det, option, code = c.determine(utterance, robot_state)
            if suc_det:
                # We successfully determiend it. We want to separate
                # 'truly' determined values from other methods (e.g.
                # inferred from robot state, default.)
                if code == Clarifier.DEFAULT:
                    # Used default value.
                    defaults += [option]
                elif code == Clarifier.INFERRED:
                    # Inferred from robot state.
                    inferred += [option]
                else:
                    # Determined from speech.
                    options += [option]
            else:
                all_clarified = False
                clarify_codes.add(code)

        # Debug printing
        Debug.p('CommandParser.parse (' + self.get_keyword() + ') got:')
        Debug.pl(1, 'keyword: ' + self.get_keyword())
        Debug.pl(1, 'options: ' + ' '.join([str(o) for o in options]))
        Debug.pl(1, 'inferred: ' + ' '.join([str(i) for i in inferred]))
        Debug.pl(1, 'defaults: ' + ' '.join([str(d) for d in defaults]))
        Debug.pl(1, 'codes: ' + ' '.join([str(cc) for cc in clarify_codes]))

        # Check result
        if all_clarified:
            # We clarified everything: build the command and return.
            return self.command_builder(
                options + inferred + defaults), clarify_codes
        else:
            # Some options remain unclarified; clarify them.
            return (
                self.clarification_builder(
                    options, inferred, defaults, clarify_codes),
                clarify_codes
            )


class Parser:
    '''string -> Command.

    This is the only object that maintains state. The state it maintains
    is what needs to be clarified.'''

    def __init__(self):
        # Make clarifiers as these are reused.
        clarifier_lr = Clarifier(
            [Side.LEFT, Side.RIGHT],  # options
            None,  # default
            Clarifier.LEFT_OR_RIGHT  # clarify_code
        )
        clarifier_rotate_dir = Clarifier(
            [Rotation.CLOCKWISE, Rotation.COUNTERCLOCKWISE],  # options
            Rotation.CLOCKWISE,  # default
            Clarifier.CW_OR_CCW  # clarify_code
        )

        # Make command parsers
        self.command_parsers = [
            # rotate wrist parser
            CommandParser(
                'rotate',  # keyword
                [clarifier_lr, clarifier_rotate_dir],  # clarifiers
                Command.build_rotate_wrist_command,  # cmd bldr
                Clarification.clarify_rotate_wrist_command  # clrfy bldr
            ),
            # open gripper parser
            CommandParser(
                'open',  # keyword
                [clarifier_lr],  # clarifiers
                Command.build_open_gripper_command,  # cmd bldr
                Clarification.clarify_open_gripper_command  # clrfy bldr
            ),
        ]

    def parse(self, utterance, robot_state=None):
        # Preprocess
        utterance = utterance.lower()
        # TODO: add branch for clarification route. Can override with
        # command.

        # Figure out how many commands the user might have been
        # specifying.
        cmd_parsers = [
            cp for cp in self.command_parsers
            if cp.get_keyword() in utterance
        ]
        # If there are none, ask user to rephrase.
        if len(cmd_parsers) == 0:
            return Clarification.NO_KEYWORD

        # If there are multiple, we have an ambiguous utterance.
        if len(cmd_parsers) > 1:
            return self.clarify(cmd_parsers)

        # There's only one---left it handle it.
        retstr, clarify_codes = cmd_parsers[0].parse(
            utterance, robot_state)

        # TODO(mbforbes): I think I don't need to intersect the
        # previous codes, just use the new ones. This may have to
        # change depending on how we use the clarification codes
        # above.
        clarify_codes = clarify_codes

        return retstr

    def clarify_cps(self, cmd_parsers):
        return self.clarify_kws([cp.get_keyword() for cp in cmd_parsers])

    def clarify_kws(self, keywords):
        beg = 'Sorry, do you want me to '
        # a, b, c, ..., or d
        mid = ', or '.join([', '.join(keywords[:-1]), keywords[:-1]])
        end = '?'
        return ''.join([beg, mid, end])

# Quick way to test.
if __name__ == '__main__':
    p = Parser()
    while True:
        utterance = raw_input('Enter sentence to parse: ')
        print p.parse(utterance)
