'''Frontends for the hands-free pbd parser.

    - Frontend:    Access parser, no-frills. Can buffer & grab logs.
    - ROSFrontend: Add option to enable ROS capabilities.
    - WebFrontend: Frontend for web interface. ROS-enabled, if desired.
    - CLFrontend:  Command-line interface. ROS-enabled, if desired.
'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import sys
import yaml

# Local
from constants import C
from grammar import ObjectOption
from hybridbayes import Parser
from roslink import WorldObject, Robot
from util import Logger, Debug, Info, Error


########################################################################
# Classes
########################################################################

class Frontend(object):
    '''
    Basic functionality.
    '''
    def __init__(self, buffer_printing=False):
        Logger.buffer_printing = buffer_printing
        self.parser = Parser()

        # Initialize for clarity
        self.start_buffer = ''
        self.parse_buffer = ''

    def parse(self, utterance):
        '''
        Parses and returns result.

        Returns:
            RobotCommand
        '''
        rc = self.parser.parse(utterance)
        self.parse_buffer = Logger.get_buffer()
        return rc

    def get_buffer(self):
        '''
        Returns the buffer from grammar generation as well as the last
        parse.

        Returns:
            str
        '''
        return '\n'.join([self.start_buffer, self.parse_buffer])

    def set_default_world(self):
        '''
        Sets "default" (file-specified) world objects and robot.
        '''
        world_dict = yaml.load(open(C.world_default))
        w_objects = WorldObject.from_dicts(world_dict['objects'])
        robot = Robot(world_dict['robot'])
        self.set_world(w_objects, robot)

    def set_world(self, world_objects=[], robot=Robot()):
        '''
        Updates the objects in the world and the robot.

        Args:
            world_objects ([WorldObject], optional): Defaults to []
            robot ([Robot], optional): Defaults to Robot().
        '''
        self._set_world_internal(world_objects, robot)

    def update_objects(self, world_objects=[]):
        '''
        Updates only the objects in the world.

        Args:
            world_objects ([WorldObject], optional): Defaults to []
        '''
        self._set_world_internal(world_objects, None)

    def update_robot(self, robot=Robot()):
        '''
        Updates only the robot.

        Args:
            robot ([Robot], optional): Defaults to Robot().
        '''
        self._set_world_internal(None, robot)

    def _set_world_internal(self, world_objects, robot):
        '''
        Sets the parser's world, robot, parser maybe regenerates.

        This is so we can capture the log for reporting (if desired).

        Args:
            world_objects ([WorldObject])
            robot ([Robot])
        '''
        self.parser.set_world(world_objects, robot)
        self.start_buffer = Logger.get_buffer()


class ROSFrontend(Frontend):
    '''Adds ROS compatability to interface (if desired).'''

    # Override functions -----------------------------------------------

    def __init__(self, buffer_printing=False):
        super(ROSFrontend, self).__init__(buffer_printing)

        # Initialize (for clarify)
        self.hfcmd_pub = None
        self.ros_running = False

    def parse(self, utterance):
        # Parse as normal
        rc = super(ROSFrontend, self).parse(utterance)

        # Maybe publish to ROS!
        if self.ros_running and self.hfcmd_pub is not None:
            self.hfcmd_pub.publish(rc.to_rosmsg())

        # And finally return
        return rc

    # New functions ----------------------------------------------------

    def startup_ros(self, spin=False):
        '''ROS-specific: Sets up callbacks for
            - recognized speech from pocketsphinx
            - world state updates
            - robot state updates
        and publishers for
            - HandsFreeCommand

        Returns:
            bool: Whether the setup succeeded. Note that this won't
            return until ros has shutdown if spin (== True)!
        '''
        # Some settings
        Debug.printing = False

        # Setup default system.
        self.set_world()

        # Setup ROS.
        try:
            import roslib
            roslib.load_manifest('pr2_pbd_interaction')
            import rospy
            from std_msgs.msg import String
            from pr2_pbd_interaction.msg import (
                HandsFreeCommand, WorldObjects, RobotState)
            # TODO(mbforbes); This waits for ROS. This is annoying, but
            # actually may be OK for now.
            rospy.init_node('hfpbd_parser')

            # We get: speech, world objects, robot state.
            rospy.Subscriber('recognizer/output', String, self.sphinx_cb)
            rospy.Subscriber(
                'handsfree_worldobjects', WorldObjects, self.world_objects_cb)
            rospy.Subscriber(
                'handsfree_robotstate', RobotState, self.robot_state_cb)

            # We send: parsed commands.
            self.hfcmd_pub = rospy.Publisher(
                'handsfree_command', HandsFreeCommand)

            # Setup complete
            self.ros_running = True

            # If no other frontend, just wait here.
            if spin:
                rospy.spin()
            return True
        except ImportError:
            # We don't have ROS installed! That's OK.
            return False

    def sphinx_cb(self, recognized):
        '''ROS-specific: Callback for when data received from
        Pocketsphinx.

        Args:
            recognized (String)
        '''
        # Ensure we actually got something.
        recog_str = recognized.data
        if len(recog_str.strip()) == 0:
            return

        # Parse; ROS response happens in parser automatically.
        robotCommand, buf_log = self.parse(recog_str)

    def world_objects_cb(self, world_objects):
        '''ROS-specific: Callback for when world objects are received
        from the robot.

        Args:
            world_objects (WorldObjects)
        '''
        self.update_objects(WorldObject.from_ros(world_objects))

    def robot_state_cb(self, robot_state):
        '''ROS-specific: Callback for when robot state is received from
        the robot.

        Args:
            robot_state (RobotState)
        '''
        self.update_robot(Robot.from_ros(robot_state))


class WebFrontend(ROSFrontend):
    '''Frontend for the web interface.'''

    # Override functions -----------------------------------------------

    def __init__(self):
        # We want to buffer printing for the web!
        super(WebFrontend, self).__init__(buffer_printing=True)

    # New functions ----------------------------------------------------

    def get_world_objects_str(self):
        '''
        For display (e.g. web interface).

        Returns:
            str
        '''
        ret = ''
        if self.parser.world_objects is not None:
            for obj in self.parser.world_objects:
                ret += obj.to_dict_str()
        return ret

    def get_robot_str(self):
        '''
        For display (e.g. web interface).

        Returns:
            str
        '''
        ret = ''
        if self.parser.robot is not None:
            ret = self.parser.robot.to_dict_str()
        return ret


class CLFrontend(ROSFrontend):
    '''Command-line frontend for the parser.'''

    def run_default_query(self):
        '''
        Programmatically runs hardcoded query. Useful for debugging.
        '''
        # Provide initial world, robot
        self.set_default_world()
        utterance = 'move right hand up'
        Info.p(self.parse(utterance))

    def simple_interactive_loop(self):
        '''
        Answers queries using no world objects and no robot state.
        '''
        self.set_world()
        self.interactive_loop()

    def default_interactive_loop(self):
        '''
        Answers queries using file-saved world objects and robot state.
        '''
        # Provide initial world, robot
        self.set_default_world()
        self.interactive_loop()

    def interactive_loop(self):
        '''
        Answers queries using the already-set world objects and robot
        state.
        '''
        while True:
            utterance = raw_input('> ')
            Info.p(self.parse(utterance))

    def print_sentences(self):
        '''Prints all sentences to stdout, one per line.'''
        Debug.printing = False
        Info.printing = False

        # NOTE(mbforbes): Because sentences can only be generated after
        # we know about the objects that we have in the world, if we
        # want to truly exhaustively generate all possible sentences
        # here then we must exhaustively generate all possible objects
        # first. HOWEVER, with word skipping for objects (you don't
        # always want to have to say all descriptors for an object), the
        # space of possible sentences explodes (into the millions). So
        # we seprately generate all sentences without objects, then
        # separately generate object phrases.
        self.set_world()
        for sentence in self.parser.sentences:
            print sentence.get_raw()

        # Debug
        wobjs = [WorldObject(o) for o in WorldObject.gen_objs()]

        # Look for object options
        sentence_set = set()
        self.set_world(world_objects=wobjs)
        for opt in self.parser.options:
            if type(opt) == ObjectOption:
                phrase_lists = opt.get_phrases(True)
                for phrase_list in phrase_lists:
                    sentence_set.add(' '.join([str(p) for p in phrase_list]))
        for sentence in sentence_set:
            print sentence

        # Backup as sentence generation with objects and word skipping
        # explodes.
        for phrase in self.parser.phrases:
            print phrase

    def main(self, args=[]):
        if args == []:
            self.run_default_query()
        else:
            arg = args[0]
            if arg == 'interactive':
                self.default_interactive_loop()
            elif arg == 'interactive-simple':
                self.simple_interactive_loop()
            elif arg == 'sentences':
                self.print_sentences()
            elif arg == 'ros':
                self.startup_ros(spin=True)
            else:
                Error.p("Unknown option: " + arg)

if __name__ == '__main__':
    clfrontend = CLFrontend()
    clfrontend.main(sys.argv[1:])
