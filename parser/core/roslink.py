'''Contains the 'link' to ROS: the objects that convert and hold state
to and from ROS formats.'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import yaml

# Local
from constants import C
from grammar import ObjectOption
from util import Debug, Info, Error, Algo


########################################################################
# Classes
########################################################################

class PropertyGetter(object):
    '''
    A safe (ish) interface for a dictionary.

    Basically allows checking / gettting properties through an API
    rather than directly.
    '''

    def __init__(self, properties=None):
        '''
        Args:
            properties ({str: object}, optional): Mapping of names to
                properties, which are usually strings, bools, or
                two-element bool lists (for right, left hands). This
                will likely come from a yaml-loaded dictionary in
                basic testing, a programatiicaly-created dictionary in
                programtic testing, and the real robot (via a ROS
                message) in real-robot testing.
        '''
        if properties is None:
            properties = {}
        self.properties = properties

    def has_property(self, name):
        '''
        Returns whether world object has a property.

        This is really mostly useful for testing, where we might omit
        some world object state.

        Returns:
            bool
        '''
        return name in self.properties

    def get_property(self, name):
        '''
        Gets a world object's property by name.

        Returns:
            object
        '''
        return self.properties[name]

    def to_dict_str(self):
        '''
        For display (e.g. in web interface).

        Returns:
            str
        '''
        return yaml.dump(self.properties)


class WorldObject(PropertyGetter):
    '''
    Contains data about the properties of an object in the world.

    Can be robot or YAML-loaded.
    '''

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return (
            self.get_property('name') if self.has_property('name')
            else 'unknownObj')

    @staticmethod
    def from_dicts(objs):
        '''
        Args:
            objs ([{str: object}]): List of dicts, each with a mapping
                from property name to value.

                This will likely be the YAML-loaded 'objects' component
                of world dict for basic testing, programmatically-
                constructed for programtic testing, and robot-sensor-
                supplied for real robot usage.

        Returns:
            [WorldObject]
        '''
        return [WorldObject(obj_dict) for obj_dict in objs]

    @staticmethod
    def from_ros(world_objects):
        '''
        Args:
            world_objects (WorldObjects):ROS-msg format WorldObjects.

        Returns:
            [WorldObject]: Array of our own WorldObject format.
        '''
        wobjs = []
        # rwo = ros world object
        for rwo in world_objects.world_objects:
            # Each object here is a WorldObject (as in WorldObject.msg).
            # This was kind of a bad decision in class names, sorry.
            # However, we're not explicitly importing it, so it should
            # be fine.
            #
            # Our task for each object from ROS is to see if a property
            # is really set. If it's not, we just won't include it in
            # our internal representation. The accessors we provide
            # check for property existence, so this is the correct way
            # (rather than copying dummy values).
            props = {}

            # Strings are probably set, but we can check.
            for op_str in C.op_strs:
                if hasattr(rwo, op_str):
                    rwo_val = getattr(rwo, op_str)
                    if len(rwo_val) > 0:
                        props[op_str] = rwo_val

            # Bools are always set, so we can't check.
            for rostype, parsertype in C.m_wo.iteritems():
                # Our world objects keep the ros type keys.
                props[rostype] = getattr(rwo, rostype)

            # Bool[] we can check.
            for parsertype, rostype in C.m_op.iteritems():
                if hasattr(rwo, rostype):
                    rwo_val = getattr(rwo, rostype)
                    if len(rwo_val) == 2:
                        props[rostype] = rwo_val

            # These are bool[]s that don't require translation.
            for op_boolarr in C.op_boolarrs:
                if hasattr(rwo, op_boolarr):
                    rwo_val = getattr(rwo, op_boolarr)
                    if len(rwo_val) == 2:
                        props[op_boolarr] = rwo_val

            wobjs += [WorldObject(props)]
        return wobjs

    @staticmethod
    def gen_objs():
        '''For sentence generation (for speech recognition training
        data), must get exhaustive list of objs.
        '''
        all_opts = []
        for k, prop in yaml.load(
                open(C.command_grammar))['descriptors'].iteritems():
            opts = [(k, o) for o in prop['options']]
            all_opts += [opts]

        all_combs = Algo.gen_recursive(all_opts)

        objs = []
        for idx, comb in enumerate(all_combs):
            obj = {'name': 'obj' + str(idx)}
            # Put in this combination of exclusive properties
            for name, val in comb:
                if val in C.m_wo_inverse:
                    # We need to map to some world object key.
                    obj[C.m_wo_inverse[val]] = val
                else:
                    # The key from the grammar was the one we use.
                    obj[name] = val
            objs += [obj]
        return objs


class Robot(PropertyGetter):
    '''
    Provides an interface for accessing robot data.
    '''

    @staticmethod
    def from_ros(robot_state):
        '''
        Args:
            robot_state (RobotState): From ROS.

        Returns:
            Robot
        '''
        props = {}

        # Uncheckable properties (e.g. bools).
        for rosname in C.rp_bools:
            props[rosname] = getattr(robot_state, rosname)

        # Strings (must have characters).
        for rosname in C.rp_strs:
            val = getattr(robot_state, rosname)
            if len(val) > 0:
                props[rosname] = val

        # bool[]-typed properties in map.
        for parsername, rosname in C.m_rp.iteritems():
            val = getattr(robot_state, rosname)
            if len(val) == 2:
                props[rosname] = val

        # other bool[]-typed properties
        for rosname in C.rp_bool_arrs:
            val = getattr(robot_state, rosname)
            if len(val) == 2:
                props[rosname] = val

        return Robot(props)


class RobotCommand(object):
    '''A Command wrapper that will generate objects in the forms that
    will be returned to the robot (or something close for us humans to
    read).
    '''

    def __init__(self, name, args, phrases, utterance):
        '''
        Used internally. Use a factory if you're calling this from
        outside this class.

        Args:
            name (str)
            args ([str])
            phrases ([str])
            utterance (str)
        '''
        self.name = name
        self.args = args
        self.phrases = phrases
        self.utterance = utterance

    @staticmethod
    def from_command(command, u_sentence, u):
        '''
        Factory.

        Args:
            command (Command)
            u_sentence (Sentence): What the user said, processed.
            u (str): Utterance: what the user said.

        Returns:
            RobotCommand
        '''
        # Get 'core' of command; its unique 'verb.'
        verb = command.get_name()

        # Note that we skip the first option in the list, because this
        # is the name of the command itself.
        opt_names = command.opt_str_list()[1:]

        # Get phrases by matching each option with the sentence.
        u_phrases = u_sentence.get_phrases()

        # Mark phrases as seen.
        for p in u_phrases:
            p.seen = True

        # Match w/ options.
        phrase_strs = []
        for opt_name, opt in command.option_map.iteritems():
            # For object options, we just return 'object X' and this
            # gets described dynamically at execution time.
            if type(opt) == ObjectOption:
                phrase_strs += [opt.pure_str()]
                continue

            # The following is for non-object options:
            # Strategy: return the full option phrase set that has the
            # highest number of phrase hits. Definitely do one for each
            # option.
            opt_phrase_sets = opt.get_phrases()
            # Just pick first by default, as we want something.
            best_set, best_set_score = opt_phrase_sets[0], -1
            for phrase_set in opt_phrase_sets:
                set_score = 0
                for phrase in phrase_set:
                    if phrase.seen:
                        set_score += phrase.get_match_score()
                if set_score > best_set_score:
                    best_set_score = set_score
                    best_set = phrase_set
            # Add best.
            phrase_strs += [' '.join([str(p) for p in best_set])]

        # Unmark phrases.
        for p in u_phrases:
            p.seen = False

        return RobotCommand(verb, opt_names, phrase_strs, u)

    @staticmethod
    def from_strs(name, args, phrases=[], utterance=''):
        '''
        Factory.

        Args:
            name (str)
            args ([str])
            phrases ([str])
            utterance (str)

        Returns:
            RobotCommand
        '''
        return RobotCommand(name, args, phrases, utterance)

    def to_rosmsg(self):
        '''
        Returns ROS representation of this command.
        '''
        from pr2_pbd_interaction.msg import HandsFreeCommand
        return HandsFreeCommand(
            cmd=self.name,
            args=self.args,
            phrases=self.phrases,
            utterance=self.utterance
        )

    def __eq__(self, other):
        '''
        Args:
            other (RobotCommand)

        Returns:
            bool
        '''
        # NOTE(mbforbes): I think we don't compare phrases, as when
        # we're checking RobotCommand's we're looking for 'execution'
        # sameness and not 'lanauge' sameness.
        return self.name == other.name and self.args == other.args

    def __ne__(self, other):
        '''
        Args:
            other (RobotCommand)

        Returns:
            bool
        '''
        return not self.__eq__(other)

    def __repr__(self):
        '''
        Returns:
            str
        '''
        return ': '.join([self.name, ', '.join(self.args)])
