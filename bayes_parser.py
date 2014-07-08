'''Bayes-based parser. Uses grammar YAML file.

Feature list:
    - Basic dialog for clarification (I think it may be best to
        implement this by having the verb phrase for each block contain
        a unique list of template/primitive blocks, and for those to all
        remember state.

    - Prior: p(gipper|open) should be 0.9, not just p(gripper)

    - Remember previous state for querying top-level commad. If the user
        specified options for something (e.g. 'counter-clockwise') then
        we should remember that when we ask for the command. This kind
        of goes with the next one.
'''

__author__ = 'mbforbes'


# Imports
########################################################################

# Builtins
from collections import defaultdict, OrderedDict
import yaml


# Constants
########################################################################

# Global options
DEBUG_PRINTING_DEFAULT = False

# The command returned to the robot when there is clarification needed.
CLARIFY_COMMAND = 'clarify'


# Classes
########################################################################

class VerbPhrase:

    # Constants for composing blocks to use.
    TEMPLATE = 1
    PRIMITIVE = 2

    def __init__(
            self, name, id_, components, template_blocks, primitive_blocks):
        self.name = name
        self.id_ = id_
        self.components = components
        self.template_blocks = template_blocks
        self.primitive_blocks = primitive_blocks

    def _clarification_needed(self, results):
        '''
        Args:
            results: n X 4 list of:
             1  [int (VerbPhrase.*), status_code, [str] (words), weight]
             2  [...,              , ...        ,              , ...   ]
                ...
             n  [...,              , ...        ,              , ...   ]

        Returns:
            bool: Whether any clarification needed
        '''
        # See if any TemplateBlocks need clarificaiton.
        for block in results:
            if block[0] == VerbPhrase.TEMPLATE:
                block_type, status_code, options, weight = block
                if status_code == TemplateBlock.CLARIFY:
                    # Clarification needed on at least one template:
                    return True
        # No templates required clarification---great!
        return False

    def _make_clarification(self, results):
        '''
        Args:
            results: n X 4 list of:
             1  [int (VerbPhrase.*), status_code, [str] (words), weight]
             2  [...,              , ...        ,              , ...   ]
                ...
             n  [...,              , ...        ,              , ...   ]

        Returns:
            str: Message to tell user about clarification
        '''
        response_pieces, clarify_pieces = [], []
        for block in results:
            if block[0] == VerbPhrase.TEMPLATE:
                block_type, status_code, options, weight = block
                # We don't report default values.
                if status_code == TemplateBlock.OK:
                    # If option OK (heard), should be only one.
                    assert len(options) == 1
                    response_pieces += [options[0]]
                # For the first clarify block, we need to list the
                # options.
                elif (status_code == TemplateBlock.CLARIFY and
                        len(clarify_pieces) == 0):
                    clarify_pieces = options[:]
            elif block[0] == VerbPhrase.PRIMITIVE:
                block_type, status_code, word_pair, weight = block
                # Whether the primitive block is known (implicit),
                # unknown (needed / core to the command), or seen
                # (actually said by the user), we know that it belongs
                # in this verb phrase.
                #
                # Remember the second of the word pair is the
                # user-selected one.
                response_pieces += [word_pair[1]]

        # Now make response.
        pre = 'I heard: '
        mid = (
            ' '.join(response_pieces) if len(response_pieces) > 1
            else response_pieces)
        clarify_pre = '. Your options are: '
        clarify = ' or '.join(clarify_pieces)
        return ''.join([pre, mid, clarify_pre, clarify])

    def _make_command(self, results):
        '''
        Args:
            results: n X 4 list of:
             1  [int (VerbPhrase.*), status_code, [str] (words), weight]
             2  [...,              , ...        ,              , ...   ]
                ...
             n  [...,              , ...        ,              , ...   ]

        Returns: tuple of
            str: command in robot form,
            str: command in human-responsive form (uses synonyms)
        '''
        # We craft separately a command to be issued to the robot (the
        # cannonical version of the command) and a response to be said
        # to the user that uses any synonyms they may have.
        command_robot_pieces, command_response_pieces = [], []
        for block in results:
            block_type = block[0]
            if block_type == VerbPhrase.TEMPLATE:
                __, status_code, options, weight = block
                # Should have only one (selected) option as this method
                # requires we don't have a clarification flag.
                assert len(options) == 1
                command_robot_pieces += [options[0]]
                command_response_pieces += [options[0]]
            elif block_type == VerbPhrase.PRIMITIVE:
                __, status_code, word_pair, weight = block
                # First is name, second synonym
                command_robot_pieces += [word_pair[0]]
                command_response_pieces += [word_pair[1]]
            else:
                raise ValueError("Unknown block type: " + str(block_type))
        return (
            ' '.join(command_robot_pieces), ' '.join(command_response_pieces))

    def _make_response(self, results):
        '''
        Args:
            results: n X 4 list of:
             1  [int (VerbPhrase.*), status_code, [str] (words), weight]
             2  [...,              , ...        ,              , ...   ]
                ...
             n  [...,              , ...        ,              , ...   ]

        Returns: tuple of:
            str: command to robot,
            str: human-readable interpretation of command (e.g. clarify
                request or command with user-used synonyms in it)
        '''
        # First determine what kind of response we're going to try to
        # make.
        #
        # reference:
        #
        # TemplateBlock:
        #     DEFAULT
        #     OK
        #     CLARIFY
        #
        # PimitiveBlock:
        #     KNOWN
        #     SEEN
        #     UNSEEN
        if self._clarification_needed(results):
            return 'clarify', self._make_clarification(results)
        else:
            return self._make_command(results)

    def generate_commands(self):
        '''Generates all commands possible for the given verb.

        Returns:
            [str]
        '''
        commands = []
        cnames = [c['name'] for c in self.components]
        tcs = [
            self.template_blocks[k] for k in cnames
            if k in self.template_blocks]
        pcs = [
            self.primitive_blocks[k] for k in cnames
            if k in self.primitive_blocks]
        components = tcs + pcs

        for block in components:
            words = block.expand_command()
            if len(commands) == 0:
                commands = words
            else:
                # Append permutations
                new_commands = []
                for command in commands:
                    for word in words:
                        new_commands += [' '.join([command, word])]
                commands = new_commands
        return new_commands

    def eval(self, utterance):
        '''
        See how likely the verb phrase is by evaluating an utterance
        over all of the templates and primitives contained within it.

        Args:
            utterance ([str])

        Returns: tuple of:
            float: P(utterance),
            str: command to robot
            str: speech for robot to say to user
        '''
        # Debugging
        Debug.pl(0, 'Trying verb: ' + self.name)
        Debug.pl(0, 'Components: ' + ', '.join(
            str(item) for item in self.components))

        # Cycle through all components
        # TODO(mbforbes): Maybe remove weights once other results fully
        # implemented.
        weights = []
        results = []
        for component in self.components:
            cname = component['name']
            Debug.pl(1, 'Trying component: ' + cname)

            # Find the template or primitive with the name & eval it.
            if cname in self.template_blocks:
                # It's a template.
                status_code, options, probability = self.template_blocks[
                    cname].eval(utterance)
                weights += [probability]
                results += [[
                    VerbPhrase.TEMPLATE, status_code, options, probability]]
            elif cname in self.primitive_blocks:
                # It's a primitive.
                known = 'known' in component and component['known']
                status_code, word_pair, probability = self.primitive_blocks[
                    cname].eval(utterance, known)
                weights += [probability]
                results += [[
                    VerbPhrase.PRIMITIVE, status_code, word_pair, probability]]
            else:
                raise ValueError("Can't find component with name: " + cname)

        # Multiply all weights together.
        #
        # TODO(mbforbes): This is probably not fully correct. Need
        # to figure out what kind of Bayesian tree this is and what
        # kind of inference we need to do.
        #
        # NOTE(mbforbes); Will likely switch to numpy or
        # scikit-learn if / when this gets more complex.
        prod = reduce(lambda x, y: x * y, weights)

        # the status code and options.
        robot_command, robot_speech = self._make_response(results)

        # Return
        return prod, robot_command, robot_speech


class TemplateBlock:
    '''Template blocks are slots that can be filled with one of several
    words (options).

    For example 'side' can be filled with 'right' or 'left'.

    The options are considered primitives and are not further expanded by
    primitive blocks (as of now).
    '''
    # Status codes

    # Default value used
    DEFAULT = -1

    # Everything OK (option picked).
    OK = 0

    # Clarification needed.
    CLARIFY = 1

    def __init__(self, name, options, default=None):
        self.name = name
        self.options = options
        self.default = default

    def expand_command(self):
        '''
        Expand the template block for all options for *commands*
        (commands to the robot). This includes all options.

        Return:
            [str]: all options for the template block.
        '''
        return self.options[:]  # Returns shallow copy

    def eval(self, utterance):
        '''
        Args:
            utterance ([str])

        Returns: tuple of:
            int: Status code (TemplateBlock.*)
            [str]: list with
                - one element (option selected) if this could be
                    determined or the default value was used
                - multiple elements if there were multiple inputs or no
                    input (and so the option could not be determined)
            float: (Independent) probability of this block.
        '''
        # The parsing we do just looks at the utterance right now. In
        # the future, we'll incorporate robot/world state & history.
        options = []
        for word in self.options:
            if word in utterance:
                options += [word]

        if len(options) > 0:
            # We got at least one option
            if len(options) == 1:
                # We got exactly one option (this is desired).
                ret_weight = Grammar.weights['seen']
                ret_code = TemplateBlock.OK
            else:
                # We got multiple options (this is undesired).
                ret_weight = Grammar.weights['multiple']
                ret_code = TemplateBlock.CLARIFY
        else:
            # No options gotten.
            if self.default is not None:
                # We have a default; use it.
                options = [self.default]
                ret_weight = Grammar.weights['default']
                ret_code = TemplateBlock.DEFAULT
            else:
                # Nothing gotten & no default: return them all.
                options = self.options[:]  # Shallow copy of options list
                ret_weight = Grammar.weights['unseen']
                ret_code = TemplateBlock.CLARIFY

        # Some debug printing...
        Debug.pl(2, 'Template: ' + self.name)
        Debug.pl(3, '- ret_code: ' + str(ret_code))
        Debug.pl(3, '- options: ' + ', '.join(options))
        Debug.pl(3, '- ret_weight: ' + str(ret_weight))

        # Give back what we determined.
        return ret_code, options, ret_weight


class PrimitiveBlock:
    '''Primitive blocks mark end words and their synonyms.'''

    # Status codes

    # Word unseen, but known.
    KNOWN = -1

    # Word seen (all OK).
    SEEN = 0

    # Word unseen and unknown (needed).
    UNSEEN = 1

    def __init__(self, name, synonyms):
        self.name = name
        self.synonyms = synonyms

    def expand_command(self):
        '''
        Expand the primitive block for all options for *commands*
        (commands to the robot). For primitives, we always use the
        name field.

        Return
            [str]: List with one element: the name of the primitive
        '''
        return [self.name]

    def eval(self, utterance, known):
        '''
        Args:
            utterance ([str])
            known (bool): Whether this word is already known in the
                phrase, i.e. doesn't need to be seen.

        Returns: tuple of:
            int: Status code (one of PrimitiveBlock.*)
            [str]: List of two strings:
                - self.name
                - the synonym that was seen. If nothing was seen, or the
                    self.name was seen, this is also self.name.
            float: (Independent) probability of this block.
        '''
        # Check all synonyms.
        seen_synonym = self.name
        status_code = PrimitiveBlock.UNSEEN
        for word in self.synonyms:
            if word in utterance:
                # Got it.
                weight = Grammar.weights['seen']
                seen_synonym = word
                status_code = PrimitiveBlock.SEEN
                break
        # Set some default weight if it's not seen.
        if not status_code == PrimitiveBlock.SEEN:
            if known:
                # We 'know' the word should be part of the command, so
                # it's not needed.
                weight = Grammar.weights['known']
                status_code = PrimitiveBlock.KNOWN
            else:
                # We didn't know the word should exist; it was just
                # unseen.
                weight = Grammar.weights['unseen']
                status_code = PrimitiveBlock.UNSEEN

        # Some debug printing...
        Debug.pl(2, 'Template: ' + self.name)
        Debug.pl(3, '- ret_weight: ' + str(weight))

        # Give back what we determined.
        return status_code, [self.name, seen_synonym], weight


class Grammar:
    '''Grammar holds grammar as defined in file.'''

    # We're keeping this separate so it can be changed globally.
    weights = None

    def __init__(self, filename='grammar.yml'):
        # Load from file
        with open(filename) as gfile:
            self.raw_grammar = yaml.load(gfile)

        # Do some basic checking
        Grammar._check_grammar(self.raw_grammar)

        # Load weights
        Grammar.weights = self.raw_grammar['weights']

        # Build up primitives
        self.primitive_blocks = OrderedDict()
        for primitive in self.raw_grammar['primitives']:
            name = primitive['name']
            synonyms = primitive['synonyms']
            self.primitive_blocks[name] = PrimitiveBlock(name, synonyms)

        # Build up templates
        self.template_blocks = OrderedDict()
        for template in self.raw_grammar['templates']:
            name = template['name']
            options = template['options']
            default = template['default'] if 'default' in template else None
            self.template_blocks[name] = TemplateBlock(name, options, default)

        # Build up verbs
        self.verb_blocks = OrderedDict()
        for verb in self.raw_grammar['verbs']:
            name = verb['name']
            id_ = verb['id']
            components = verb['components']
            self.verb_blocks[name] = VerbPhrase(
                name,
                id_,
                components,
                self.template_blocks,
                self.primitive_blocks
            )

        # As a debug, generate all possible commands and print them.
        all_commands = self.generate_commands()
        Debug.p('All possible commands:')
        for command in all_commands:
            Debug.pl(1, command)

    def eval(self, utterance):
        '''
        Args:
            utterance ([str])

        Returns: tuple of:
            float: probability,
            str: robot command,
            str: robot speech
        '''
        results_dict = defaultdict(list)
        for verb, verb_block in self.verb_blocks.iteritems():
            prob, robot_command, robot_speech = verb_block.eval(utterance)
            results_dict[prob] += [(robot_command, robot_speech)]
        # Pull off with increasing probability
        last_prob = 0.0
        while len(results_dict) > 0:
            last_prob = min(results_dict)
            items = results_dict.pop(last_prob)
            for item in items:
                Debug.p(' '.join([str(last_prob), str(item)]))
        # Return the last (the most likely)
        return last_prob, item[0], item[1]

    def generate_commands(self):
        '''Generates all possible commands this grammar could send to
        the robot.

        Returns:
            [str]: List of all strings that can be returned as robot
                commands.
        '''
        all_commands = []
        for verb_name, verb_block in self.verb_blocks.iteritems():
            all_commands += verb_block.generate_commands()
        return all_commands

    @staticmethod
    def _check_grammar(raw_grammar):
        '''Does basic checking that the grammar file isn't messed up
        (like whether it contains duplicate or missing template /
        primitive names).
        '''
        # Check all template & primitive names unique
        check_sections = ['templates', 'primitives']
        all_blocks = set()
        for section in check_sections:
            for block in raw_grammar[section]:
                size_before = len(all_blocks)
                all_blocks.add(block['name'])
                assert size_before + 1 == len(all_blocks)

        # Check all components referenced by veb phrases exist.
        for verb_phrase in raw_grammar['verbs']:
            for component in verb_phrase['components']:
                assert component['name'] in all_blocks

        Debug.p('Basic grammar check OK.')


# Most things are handled by the grammar, but this is "the parser".
class BayesParser:
    '''The parser itself.'''

    def __init__(self, grammar_filename='grammar.yml'):
        self.grammar = Grammar(grammar_filename)

    def _parse(self, utterance):
        '''
        Returns: tuple of:
            int: prob,
            str: command,
            str: speech
        '''
        # Preprocess into array (easier 'word in' testing)
        utterance = utterance.lower().split(' ')
        Debug.p('Processed utterance:' + str(utterance))

        # Eval with grammar
        return self.grammar.eval(utterance)

    def parse_return_command(self, utterance):
        '''
        Primarily for testing / real use.
        Returns:
            str: command sent to robot
        '''
        return self._parse(utterance)[1]

    def parse_return_speech(self, utterance):
        '''
        For interactive testing (debugging / demonstration).

        Returns:
            str: speech robot says to human
        '''
        return self._parse(utterance)[2]


# Debugging
class Debug:
    '''Easy debug printing with optional indentation (tab) level.

    Use Debug.printing as a global switch.
    '''
    printing = DEBUG_PRINTING_DEFAULT

    @staticmethod
    def p(obj):
        Debug.pl(0, obj)

    @staticmethod
    def pl(level, obj):
        '''
        Args:
            level (int): how many tabs (one space if 0)
            string (obj): obj (maybe string) to print
        '''
        if Debug.printing:
            indent = ' ' if level == 0 else '\t' * (level + 1)
            print ''.join(['[DEBUG]', indent, str(obj)])


# Program enters here if run.
if __name__ == '__main__':
    # Simple loop for hand-testing.
    bp = BayesParser()
    print 'Welcome to the Bayes hfpbd parser.'
    while True:
        res = bp.parse_return_speech(raw_input('> '))
        print ''.join(['>>> ', res])
    print 'Exiting.'
