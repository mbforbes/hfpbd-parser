'''Bayes-based parser.'''


# Imports
########################################################################

import yaml


# Constants
########################################################################

# Global options
DEBUG_PRINTING_DEFAULT = True


# Classes
########################################################################

class GrammarBlock:
    '''A chunk of grammar that can be evaluated. A unit in the
    template.
    '''
    def eval(self, *args):
        raise NotImplementedError("Eval must be implemented in subclass.")


class VerbBlock(GrammarBlock):
    def __init__(
            self, name, id_, components, template_blocks, primitive_blocks):
        self.name = name
        self.id_ = id_
        self.components = components
        self.template_blocks = template_blocks
        self.primitive_blocks = primitive_blocks

    def eval(self, utterance):
        weights = []
        Debug.pl(0, 'Trying verb: ' + self.name)
        Debug.pl(0, 'Components: ' + ', '.join(
            str(item) for item in self.components))
        for component in self.components:
            cname = component['name']
            Debug.pl(1, 'Trying component: ' + cname)

            # Find the template or primitive with the name & eval it.
            if cname in self.template_blocks:
                # It's a template.
                status_code, options, probability = self.template_blocks[
                    cname].eval(utterance)
                # TODO(mbforbes): Use the status code and options.
                weights += [probability]
            elif cname in self.primitive_blocks:
                # It's a primitive.
                known = 'known' in component and component['known']
                weights += [self.primitive_blocks[cname].eval(
                    utterance, known)]
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

        # TODO(mbforbes): Use the status code and options.

        # Return val.
        return prod


class TemplateBlock(GrammarBlock):
    # Status codes

    # Default value used
    DEFAULT = -1

    # Nothing.
    NONE = 0

    # Clarification needed.
    CLARIFY = 1

    def __init__(self, name, options, default=None):
        self.name = name
        self.options = options
        self.default = default

    def eval(self, utterance):
        '''
        Args:
            utterance (str)

        Returns: tuple of:
            int: Status code (TemplateBlock.*)
            [str]: list with
                - one element (option selected) if this could be
                    determined or the default value was used
                - multiple elements if there were multiple inputs or the
                    option could not be determined
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
                ret_code = TemplateBlock.NONE
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


class PrimitiveBlock(GrammarBlock):
    def __init__(self, name, synonyms):
        self.name = name
        self.synonyms = synonyms

    def eval(self, utterance, known):
        '''
        Args:
            utterance (str)
            known (bool): Whether this word is already known in the
                phrase, i.e. doesn't need to be seen.

        Returns:
            float: (Independent) probability of this block.
        '''
        # Check all synonyms.
        seen = False
        for word in self.synonyms:
            if word in utterance:
                # Got it.
                weight = Grammar.weights['seen']
                seen = True
                break
        # Set some default weight if it's not seen.
        if not seen:
            if known:
                # We 'know' the word should be part of the command, so
                # it's not needed.
                weight = Grammar.weights['known']
            else:
                # We didn't know the word should exist; it was just
                # unseen.
                weight = Grammar.weights['unseen']

        # Some debug printing...
        Debug.pl(2, 'Template: ' + self.name)
        Debug.pl(3, '- ret_weight: ' + str(weight))

        # Give back what we determined.
        return weight


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
        self.primitive_blocks = {}
        for primitive in self.raw_grammar['primitives']:
            name = primitive['name']
            synonyms = primitive['synonyms']
            self.primitive_blocks[name] = PrimitiveBlock(name, synonyms)

        # Build up templates
        self.template_blocks = {}
        for template in self.raw_grammar['templates']:
            name = template['name']
            options = template['options']
            default = template['default'] if 'default' in template else None
            self.template_blocks[name] = TemplateBlock(name, options, default)

        # Build up verbs.
        self.verb_blocks = {}
        for verb in self.raw_grammar['verbs']:
            name = verb['name']
            id_ = verb['id']
            components = verb['components']
            self.verb_blocks[name] = VerbBlock(
                name,
                id_,
                components,
                self.template_blocks,
                self.primitive_blocks
            )

    def eval(self, utterance):
        for verb, verb_block in self.verb_blocks.iteritems():
            prob = verb_block.eval(utterance)
            print verb, prob

    @staticmethod
    def _check_grammar(raw_grammar):
        '''Does basic checking that the grammar file isn't messed up
        (like whether it contains duplicate names).
        '''
        # TODO(mbforbes): Implement this.
        pass


class BayesParser:
    '''The parser itself.'''

    def __init__(self, grammar_filename='grammar.yml'):
        self.grammar = Grammar(grammar_filename)

    def parse(self, utterance):
        # Preprocess
        utterance = utterance.lower()

        # Eval (this prints... won't later)
        self.grammar.eval(utterance)

        return 'dummy'


# Debugging
class Debug:
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
        res = bp.parse(raw_input('input > '))
        print ''.join(['output> ', res])
    print 'Exiting.'
