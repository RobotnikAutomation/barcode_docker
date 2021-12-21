#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rcomponent.rcomponent import *

# Insert here general imports:
import actionlib

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import Registers

from robotnik_navigation_msgs.msg import DockAction, DockFeedback, DockResult


class BarcodeDocker(RComponent):
    """
    Node that allows to dock the robot using two barcode scanners.
    """

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.modbus_io_sub_name = rospy.get_param(
            '~modbus_io_sub_name', 'robotnik_roller_belt_modbus_io/registers')
        self.barcode_dock_action_name = rospy.get_param(
            '~barcode_dock_action_name', 'barcode_docker')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Publisher
        #self.status_pub = rospy.Publisher(
        #    '~status', String, queue_size=10)
        #self.status_stamped_pub = rospy.Publisher(
        #    '~status_stamped', StringStamped, queue_size=10)

        # Subscriber
        self.modbus_io_sub = rospy.Subscriber(
            self.modbus_io_sub_name, Registers, self.modbus_io_sub_cb)
        RComponent.add_topics_health(self, self.modbus_io_sub, topic_id='modbus_io_sub', timeout=0.5, required=True)

        # Service
        #self.example_server = rospy.Service(
        #    '~example', Trigger, self.example_server_cb)

        # Actionlib server
        self.barcode_dock_action = actionlib.SimpleActionServer(self.barcode_dock_action_name, DockAction, None, auto_start=False)
        self.barcode_dock_action.register_goal_callback(self.barcode_dock_goal_cb)
        self.barcode_dock_action.register_preempt_callback(self.barcode_dock_preempt_cb)
        self.barcode_dock_action.start()

        return 0

    def init_state(self):
        self.running_barcode_dock = False

        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if(self.check_topics_health() == False):
            self.switch_to_state(State.EMERGENCY_STATE)
            return RComponent.ready_state(self)

        # Publish topic with status

        #status_stamped = StringStamped()
        #status_stamped.header.stamp = rospy.Time.now()
        #status_stamped.string = self.status.data

        #self.status_pub.publish(self.status)
        #self.status_stamped_pub.publish(status_stamped)

        # Publish actionlib feedback
        finished = False
        if (self.running_barcode_dock == True):

            if self.barcode_dock_action.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.barcode_dock_action_name)
                self.barcode_dock_action.set_preempted()

            finished = True
            #self.barcode_dock_feedback.header.stamp = rospy.Time.now()
            barcode_dock_feedback = DockFeedback()
            barcode_dock_feedback.remaining.x = 1.0
            self.barcode_dock_action.publish_feedback(barcode_dock_feedback)
            
            if (finished == True):
                barcode_dock_result = DockResult()
                #barcode_dock_result.header.stamp = rospy.Time.now()
                barcode_dock_result.success = True
                description = "Barcode docking finished successfully"
                barcode_dock_result.description = description
                rospy.loginfo('%s::action_goal_cb: %s.' %
                             (self._node_name, description))
                self.barcode_dock_action.set_succeeded(barcode_dock_result)
                self.running_barcode_dock = False

        return RComponent.ready_state(self)

    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def modbus_io_sub_cb(self, msg):
        try:
            self.barcode_pos_front = msg.registers[0].value
            self.barcode_pos_rear = msg.registers[1].value
            self.tick_topics_health('modbus_io_sub')
        except:
            rospy.logerr("Error reading barcode position. Is register 0 publishing?")


    def barcode_dock_goal_cb(self):
        """Action server goal callback
        Accepts the new goal if not command running.  Rejects the incoming
        goal if a command is running
        """
        if self.barcode_dock_action.is_active() == False:
            self.goal = self.barcode_dock_action.accept_new_goal()

            # Docking process not allowed if the component is not ready
            if self._state != State.READY_STATE:
                msg = 'Docking process not allowed because the component is not READY'
                rospy.logerr('%s::barcode_dock_goal_cb: %s' % (self._node_name, msg))
                result = DockResult()
                result.success = False
                result.description = msg
                self.barcode_dock_action.set_aborted(result=result, text=result.description)

            # TODO Before sending the command to the handler, it is validated
            is_valid = True
            if is_valid == True:
                self.running_barcode_dock = True
                rospy.loginfo('%s::action_goal_cb: Barcode docking process started.' %
                             (self._node_name))
            else:
                # Aborting the action
                rospy.logerr('%s::action_goal_cb: The goal is not valid.' %
                             (self._node_name))
                result = DockResult()
                result.success = False
                result.description = "The goal is not valid"

                self.barcode_dock_action.set_aborted(
                    result=result, text=result.description)

        else:
            # do nothing -> discards the command
            msg = 'Dock action is already running.'
            rospy.logwarn(
                '%s::action_goal_cb: %s', self._node_name, msg)

    def barcode_dock_preempt_cb(self):
        """Action server preempt callback
        Cancels the current active goal or ignore the incoming goal if
        the preempt request has been triggered by new goal available.
        """
        if self.barcode_dock_action.is_active():
            has_new_goal = self.barcode_dock_action.is_new_goal_available()
            # If preempt request by new action
            if has_new_goal == False:
                self.running_barcode_dock = False
                rospy.logwarn(
                    '%s::barcode_dock_preempt_cb: cancelled current docking action', self._node_name)
                result = DockResult()
                result.success = True
                result.description = "The docking action has been canceled by the user"
                self.barcode_dock_action.set_aborted(
                    result=result, text=result.description)
            else:
                rospy.logwarn(
                    '%s::barcode_dock_preempt_cb: preemption due to a new goal is not allowed', self._node_name)
        else:
            rospy.logwarn('%s::barcode_dock_preempt_cb: No dock action is running', self._node_name)
