#!/usr/bin/python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin -- ARC Lab                     #
#                                                           #
#   Displays interactive markers of valve-like objects and  #
#   allows the user to align the objects against the world  #
#   represented by the robot's sensors.                     #
#                                                           #
#############################################################

import rospy
import math
import time
import threading
import subprocess
from copy import deepcopy

from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from shape_msgs.msg import *
from transformation_helper import *
from icp_server_msgs.msg import *
from icp_server_msgs.srv import *
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *


# [X] Make each arrow remember it's own pose
# [X] Add a menu option to send the left and right grippers out
# [-] Add option to show / display arrows on the wheel
# [X] Add a call to enable / disable the entire plugin
# [-] (?) Turn off the control for the valve when moving arrows?
# [X] Confirm arrow angle movement amount
# [X] Add Option to change the length of the arrow (approach length)
# [ ] Add model of the hands to the end of each arrow
# [ ] Make it so that movement turns the disk purple
# [ ] Add a way to save trajectorie names
# [ ] Add a way to flush trajectories
# [ ] Turn valve opaque / not opaque
# [X] Add length to the message
# [X] Remove LEFT / RIGHT Options
# [X] Set it to default pose for the challenge
# [ ] Add a "Lock Marker" Option
# [ ] Make it so that move flushes the trajectories
# [ ] Add a label to the execution case
# [ ] Push-Pop for execution


class ValveStatus:

    CW = "CW"
    CCW = "CCW"
    UNSNAPPED = 'UNSNAPPED'
    SNAPPED = 'SNAPPED'
    SNAPPEDPLUS = 'SNAPPEDPLUS'

    LEFT = 'LH' #Right Lever Default
    RIGHT = 'RH' #Left Lever Default
    BOTH = 'BH' #Round Valve Default

    USER_LH = 'USER_LH' #User LH Only
    USER_RH = 'USER_RH' #User RH Only
    USER_BH = 'USER_BH' #USer Both Hands

    GRIPPER = 'GRIPPER'
    PEG = 'PEG'

    ROUND = 'W'
    LEFTLEVER = 'LL'
    RIGHTLEVER = 'RL'
    READY = 'READY'
    PLANNING = 'PLANNING'
    PLANNED = 'PLANNED'
    EXECUTING = 'EXECUTING'
    EXECUTED = 'EXECUTED'
    ERROR = 'ERROR'
    IDLE = "IDLE"
    GETREADY = "GETREADY"
    TURN = "TURNVALVE"
    GRASP = "GRASP"
    UNGRASP = "UNGRASP"
    FINISH = "END"
    PREVIEW = "PREVIEW"
    EXECUTE = "EXECUTE"
    EXECUTE_STORED = "EXECUTESTORED"
    LOCKED = "LOCKED"
    UNLOCKED = "UNLOCKED"

    def __init__(self):
        self.default_radius = 0.203
        self.default_thickness = 0.02
        self.radius = self.default_radius
        self.default_pose_stamped = PoseStamped()
        self.default_pose_stamped.header.frame_id = "/Body_TSY"
        self.default_pose_stamped.pose.position.x = .464
        self.default_pose_stamped.pose.position.y = .01
        self.default_pose_stamped.pose.position.z = -.036
        self.default_pose_stamped.pose.orientation.x = 0.0
        self.default_pose_stamped.pose.orientation.y = 0.0
        self.default_pose_stamped.pose.orientation.z = 0.0
        self.default_pose_stamped.pose.orientation.w = 1.0
        self.session_pose_stamped = deepcopy(self.default_pose_stamped)
        self.pose_stamped = deepcopy(self.default_pose_stamped)
        self.icp_status = self.UNSNAPPED
        self.end_effector = self.GRIPPER
        if self.end_effector == self.GRIPPER:
            self.hands = self.BOTH
        elif self.end_effector == self.PEG:
            self.hands = self.USER_BH
        self.valve_type = self.ROUND
        self.operating_status = self.READY
        self.turning_direction = self.CCW
        self.last_pointcloud = None
        self.planning_status = self.IDLE
        self.visible = True
        self.color = "CLEAR"
        self.lock_state = self.UNLOCKED




class TrajectoryMenu:

    NOERROR = "NOERROR"
    SAVE = "SAVE"
    FLUSH = "FLUSH"

    def __init__(self):
        self.menuOptions = set()


class GripperStatus:

    def __init__(self):
        self.init = False
        self.poseStamped = PoseStamped()
        self.gripperOffset = PoseStamped()
        self.prevFeedbackPose = Pose()
        self.name = None
        self.circ_angle = 0.0
        self.angle = 0
        self.length = .13
        self.visible = True



class ValveLocalizer:

    def __init__(self, marker_namespace, data_dir, planner_service, execution_service):
        self.marker_namespace = marker_namespace

        rospy.Subscriber("valve_localization/show", Bool, self.show_tool)

        self.trajMenu = TrajectoryMenu()

        #Setup The Valve and Valve Marker
        self.valve_status = ValveStatus()
        self.populate_valve_menu()

        #Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)

        #Setup the connection to the planning software
        rospy.loginfo("NOT Connecting to planner...")
        self.planner_client = rospy.ServiceProxy(planner_service, PlanTurning)
        #self.planner_client.wait_for_service()
        self.execution_client = rospy.ServiceProxy(execution_service, ExecuteTurning)
        #self.execution_client.wait_for_service()
        rospy.loginfo("...NOT Connected to planner")

        # Setup the Grippers / Arrows
        self.left_gripper = GripperStatus()
        self.right_gripper = GripperStatus()

        self.left_gripper.name = "left_gripper"
        self.left_gripper.circ_angle = (3 * math.pi) / 2
        self.left_gripper.poseStamped = deepcopy(self.valve_status.pose_stamped)
        self.left_gripper.prevFeedbackPose = deepcopy(self.valve_status.pose_stamped.pose)

        self.right_gripper.name = "right_gripper"
        self.right_gripper.circ_angle = math.pi / 2
        self.right_gripper.poseStamped = deepcopy(self.valve_status.pose_stamped)

        #Set Up Pegs
        self.left_peg = GripperStatus()
        self.right_peg = GripperStatus()

        self.left_peg.name = "left_peg"
        self.left_peg.poseStamped = deepcopy(self.valve_status.pose_stamped)

        self.right_peg.name = "right_peg"
        self.right_peg.poseStamped = deepcopy(self.valve_status.pose_stamped)

        #self.populate_gripper_menu()

        # Setup the Refresh Rate of the Marker
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



    def show_tool(self, data):
        if data == False:
            self.valve_status.visible = False
            self.left_gripper.visible = False
            self.right_gripper.visible = False
            self.left_peg.visible = False
            self.right_peg.visible = False

        if self.valve_status.end_effector == self.valve_status.GRIPPER:

            if data == True:
                if self.valve_status.hands == self.valve_status.USER_BH:
                    self.left_gripper.visible = True
                    self.right_gripper.visible = True

                if self.valve_status.hands == self.valve_status.USER_LH:
                    self.left_gripper.visible = True

                if self.valve_status.hands == self.valve_status.USER_RH:
                    self.right_gripper.visible = True

        if self.valve_status.end_effector == self.valve_status.PEG:

            if data == True:
                if self.valve_status.hands == self.valve_status.USER_BH:
                    self.left_peg.visible = True
                    self.right_peg.visible = True

                if self.valve_status.hands == self.valve_status.USER_LH:
                    self.left_peg.visible = True

                if self.valve_status.hands == self.valve_status.USER_RH:
                    self.right_peg.visible = True


    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()

        if self.valve_status.visible == True:
            alignment_marker = self.make_valve_imarker(self.valve_status.pose_stamped)
            self.server.insert(alignment_marker, self.alignment_feedback_cb)
            self.menu_handler.apply(self.server, "valve_alignment")

        if self.valve_status.end_effector == self.valve_status.GRIPPER:

            if self.left_gripper.init == False:
                self.left_gripper.gripperOffset.pose.position.x = self.left_gripper.length

            if self.right_gripper.init == False:
                self.right_gripper.gripperOffset.pose.position.x = self.right_gripper.length

            if self.valve_status.hands == self.valve_status.BOTH or\
               self.valve_status.hands == self.valve_status.LEFT or\
               self.valve_status.hands == self.valve_status.RIGHT:
                self.left_gripper.visible = False
                self.right_gripper.visible = False
            if self.valve_status.hands == self.valve_status.USER_BH:
                self.left_gripper.visible = True
                self.right_gripper.visible = True
            if self.valve_status.hands == self.valve_status.USER_LH:
                self.left_gripper.visible = True
                self.right_gripper.visible = False
            if self.valve_status.hands == self.valve_status.USER_RH:
                self.left_gripper.visible = False
                self.right_gripper.visible = True


            if (self.left_gripper.visible == True):
                left_gripper_marker = self.make_gripper_imarker(self.left_gripper)
                self.server.insert(left_gripper_marker, self.gripper_feedback_cb)
                #self.gripper_menu_handler.apply(self.server, self.left_gripper.name)

            if (self.right_gripper.visible == True):
                right_gripper_marker = self.make_gripper_imarker(self.right_gripper)
                self.server.insert(right_gripper_marker, self.gripper_feedback_cb)
                #self.gripper_menu_handler.apply(self.server, self.right_gripper.name)

        if self.valve_status.end_effector == self.valve_status.PEG:

            if self.left_peg.init == False:
                self.left_peg.gripperOffset.pose.position.x = self.left_peg.length
                self.left_peg.gripperOffset.pose.position.y = self.valve_status.radius / 2

            if self.right_peg.init == False:
                self.right_peg.gripperOffset.pose.position.x = self.right_peg.length
                self.right_peg.gripperOffset.pose.position.y = -self.valve_status.radius / 2

            if self.valve_status.hands == self.valve_status.BOTH or\
               self.valve_status.hands == self.valve_status.LEFT or\
               self.valve_status.hands == self.valve_status.RIGHT:
                self.left_peg.visible = False
                self.right_peg.visible = False
            if self.valve_status.hands == self.valve_status.USER_BH:
                self.left_peg.visible = True
                self.right_peg.visible = True
            if self.valve_status.hands == self.valve_status.USER_LH:
                self.left_peg.visible = True
                self.right_peg.visible = False
            if self.valve_status.hands == self.valve_status.USER_RH:
                self.left_peg.visible = False
                self.right_peg.visible = True


            if (self.left_peg.visible == True):
                left_peg_marker = self.make_peg_imarker(self.left_peg)
                self.server.insert(left_peg_marker, self.peg_feedback_cb)
                #self.peg_menu_handler.apply(self.server, self.left_peg.name)

            if (self.right_peg.visible == True):
                right_peg_marker = self.make_peg_imarker(self.right_peg)
                self.server.insert(right_peg_marker, self.peg_feedback_cb)
                #self.peg_menu_handler.apply(self.server, self.right_peg.name)


        self.server.applyChanges()




    def call_icp(self):
        if (self.valve_status.last_pointcloud == None):
            rospy.logerr("No pointcloud available for using ICP")
            return
        req = RunICPRequest()
        # Set the target pointcloud
        req.Request.TargetPointCloud = self.valve_status.last_pointcloud
        # Set the origin of the bounding box
        req.Request.UseBoundingVolume = True
        req.Request.BoundingVolumeOrigin = valve_in_sensor_frame_transform
        # Set the bounding box size
        req.Request.BoundingVolumeDimensions.x = self.valve_status.radius * 3.0
        req.Request.BoundingVolumeDimensions.y = self.valve_status.radius * 3.0
        req.Request.BoundingVolumeDimensions.z = self.valve_status.radius * 3.0
        # Set the input pointcloud
        req.Request.InputType = req.Request.SHAPE
        req.Request.SolidPrimitive = SolidPrimitive()
        req.Request.SolidPrimitive.type = SolidPrimitive.CYLINDER
        res = None
        try:
            res = self.icp_client.call(req)
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the ICP server running?")
        if (res != None):
            pose_adjustment = PoseFromTransform(res.Response.AlignmentTransform)
            self.valve_status.pose_stamped = ComposePoses(self.valve_status.pose_stamped.pose,pose_adjustment)




    def call_planner(self, stage):
        self.valve_status.planning_status = stage
        req = PlanTurningRequest()
        req.Request.ValvePose = self.valve_status.pose_stamped
        req.Request.ValveSize = self.valve_status.radius
        req.Request.EndEffector = self.valve_status.end_effector
        req.Request.Hands = self.valve_status.hands
        req.Request.ValveType = self.valve_status.valve_type
        req.Request.Direction = self.valve_status.turning_direction
        req.Request.TaskStage = self.valve_status.planning_status

        if self.valve_status.end_effector == self.valve_status.GRIPPER:
            req.Request.useLeft = self.left_gripper.visible
            req.Request.useRight = self.right_gripper.visible
            req.Request.LeftPose = self.left_gripper.poseStamped
            req.Request.RightPose = self.right_gripper.poseStamped
            req.Request.LeftLength = self.left_gripper.length
            req.Request.RightLength = self.right_gripper.length

        if self.valve_status.end_effector == self.valve_status.PEG:
            req.Request.useLeft = self.left_peg.visible
            req.Request.useRight = self.right_peg.visible
            req.Request.LeftPose = self.left_peg.poseStamped
            req.Request.RightPose = self.right_peg.poseStamped
            req.Request.LeftLength = self.left_peg.length
            req.Request.RightLength = self.right_peg.length

        res = None
        try:
            rospy.loginfo("Sending call to planner with request: " + str(req))
            res = self.planner_client.call(req)
            error_code = res.Response.ErrorCode
            labels = res.Response.Labels

            if error_code == self.trajMenu.NOERROR:
                self.valve_status.color = "GREEN"
            elif error_code == self.trajMenu.SAVE:
                self.insertTrajMenuOption(labels)
                self.valve_status.color = "GREEN"
            elif error_code == self.trajMenu.FLUSH:
                self.removeTrajMenuOption(labels)
                self.valve_status.color = "GREEN"
            elif error_code == "NoError":
                rospy.loginfo("Planner retunred with no error (" + error_code + ")")
                self.valve_status.color = "GREEN"
            elif error_code == "Happy birthday to you.":
                rospy.loginfo("Planner returned with no error (" + error_code + ")")
                self.valve_status.color = "GREEN"
            else:
                rospy.loginfo("Planner returned unknown error code: " + error_code + " | Labels: " + str(labels))
                self.valve_status.color = "RED"

            self.valve_status.operating_status = self.valve_status.PLANNED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the planning server running?")
            self.valve_status.operating_status = self.valve_status.ERROR
            self.valve_status.color = "RED"




    def call_execute(self, mode):
        req = ExecuteTurningRequest()
        req.Identifier = mode
        res = None
        try:
            rospy.loginfo("Sending execute call")
            self.valve_status.operating_status = self.valve_status.EXECUTING
            res = self.execution_client.call(req)
            error_code = res.ErrorCode
            labels = res.Labels

            if error_code == self.trajMenu.NOERROR:
                self.valve_status.color = "GREEN"
            elif error_code == self.trajMenu.SAVE:
                self.insertTrajMenuOption(labels)
                self.valve_status.color = "GREEN"
            elif error_code == self.trajMenu.FLUSH:
                self.removeTrajMenuOption(labels)
                self.valve_status.color = "GREEN"
            elif error_code == "NoError":
                rospy.loginfo("Trajectories executed with no error (" + error_code + ")")
                self.valve_status.color = "GREEN"
            elif error_code == "Happy birthday to you.":
                rospy.loginfo("Trajectories executed with no error (" + error_code + ")")
                self.valve_status.color = "GREEN"
            else:
                rospy.logerr("Trajectories executed with error code: " + error_code)
                self.valve_status.color = "RED"

            self.valve_status.operating_status = self.valve_status.EXECUTED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the execution server running?")
            self.valve_status.operating_status = self.valve_status.ERROR
            self.valve_status.color = "RED"





##################################
#   Valve Menu Stuff             #
##################################



    def populate_valve_menu(self):

        global snap_icp
        global planning_master, planning_getready, planning_turning, planning_finish, planning_preview, planning_grasp, planning_ungrasp
        global execute_master, execute, execute_stored
        global trajectory_master
        global radius_master, radius_increase, radius_decrease, radius_default
        global pose_master, pose_reset_default, pose_reset_session_defaul, pose_set_default
        global hand_master, hand_both, hand_left, hand_right, hand_user_both, hand_user_left, hand_user_right, hand_switch
        global type_master, type_round, type_left, type_right
        global direction_master, direction_cw, direction_ccw
        global lock_master

        self.menu_handler = MenuHandler()

        if self.valve_status.lock_state == self.valve_status.UNLOCKED:
            #Snap with ICP
            snap_icp = self.menu_handler.insert("Snap With ICP", callback = self.snap_icp_cb)

        #Planning Options
        planning_master = self.menu_handler.insert("Planning", callback = self.planning_cb)
        planning_preview = self.menu_handler.insert("PREVIEW Plan", parent = planning_master, callback = self.planning_cb)
        planning_getready = self.menu_handler.insert("Plan GETREADY", parent = planning_master, callback = self.planning_cb)
        planning_grasp = self.menu_handler.insert("Plan GRASP", parent = planning_master, callback = self.planning_cb)
        planning_ungrasp = self.menu_handler.insert("Plan UNGRASP", parent = planning_master, callback = self.planning_cb)
        planning_turning = self.menu_handler.insert("Plan TURNING", parent = planning_master, callback = self.planning_cb)
        planning_finish = self.menu_handler.insert("Plan FINISH", parent = planning_master, callback = self.planning_cb)

        #Execute Options
        execute_master = self.menu_handler.insert("Execute", callback = self.execute_cb)
        execute = self.menu_handler.insert("Execute", parent = execute_master, callback = self.execute_cb)
        execute_stored = self.menu_handler.insert("Execute Stored", parent = execute_master, callback = self.execute_cb)

        #Trajectory Options
        trajectory_master = self.menu_handler.insert("Cached Trajectories", callback = self.trajectory_cb)
        if len(self.trajMenu.menuOptions) == 0:
            self.menu_handler.setVisible(trajectory_master, False)
        elif len(self.trajMenu.menuOptions) >= 1:
            self.menu_handler.setVisible(trajectory_master, True)
            for option in self.trajMenu.menuOptions:
                self.menu_handler.insert(option, parent = trajectory_master, callback = self.trajectory_cb)

        #Lock Options
        if self.valve_status.lock_state == self.valve_status.UNLOCKED:
            lock_master = self.menu_handler.insert("LOCK Valve", callback = self.lock_cb)
        elif self.valve_status.lock_state == self.valve_status.LOCKED:
            lock_master = self.menu_handler.insert("UNLOCK Valve", callback = self.lock_cb)

        if self.valve_status.lock_state == self.valve_status.UNLOCKED:

            #Radius Options
            radius_master = self.menu_handler.insert("Radius", callback = self.radius_cb)
            radius_increase = self.menu_handler.insert("Increase Radius", parent = radius_master, callback = self.radius_cb)
            radius_decrease = self.menu_handler.insert("Decrease Radius", parent = radius_master, callback = self.radius_cb)
            radius_default = self.menu_handler.insert("Default Radius", parent = radius_master, callback = self.radius_cb)

            #Pose Options
            pose_master = self.menu_handler.insert("Pose", callback = self.pose_cb)
            pose_reset_default = self.menu_handler.insert("Reset Default Pose", parent = pose_master, callback = self.pose_cb)
            pose_reset_session_default = self.menu_handler.insert("Reset Session Default", parent = pose_master, callback = self.pose_cb)
            pose_set_default = self.menu_handler.insert("Set Default Pose", parent = pose_master, callback = self.pose_cb)

            #Hand Options
            if self.valve_status.valve_type == self.valve_status.ROUND:
                hand_master = self.menu_handler.insert("Hands", callback = self.hand_cb)
                if self.valve_status.end_effector == self.valve_status.GRIPPER:
                    hand_both = self.menu_handler.insert("Planner Assign Both Hands", parent = hand_master, callback = self.hand_cb)
                    hand_left = self.menu_handler.insert("Planner Assign Left Hand Only", parent = hand_master, callback = self.hand_cb)
                    hand_right = self.menu_handler.insert("Planner Assign Right Hand Only", parent = hand_master, callback = self.hand_cb)
                hand_user_both = self.menu_handler.insert("User Defines Both Hands", parent = hand_master, callback = self.hand_cb)
                hand_user_left = self.menu_handler.insert("User Defined Left Hand Only", parent = hand_master, callback = self.hand_cb)
                hand_user_right = self.menu_handler.insert("User Defined Right Hand Only", parent = hand_master, callback = self.hand_cb)

                if self.valve_status.end_effector == self.valve_status.GRIPPER:
                    hand_switch = self.menu_handler.insert("SWITCH TO USING PEGS", parent = hand_master, callback = self.hand_cb)
                elif self.valve_status.end_effector == self.valve_status.PEG:
                    hand_switch = self.menu_handler.insert("SWITCH TO USING GRIPPERS", parent = hand_master, callback = self.hand_cb)

                self.menu_handler.setCheckState(hand_both, False)
                self.menu_handler.setCheckState(hand_user_both, False)
                self.menu_handler.setCheckState(hand_user_left, False)
                self.menu_handler.setCheckState(hand_user_right, False)
                self.menu_handler.setCheckState(hand_left, False)
                self.menu_handler.setCheckState(hand_right, False)

                if self.valve_status.hands == self.valve_status.USER_BH:
                    self.menu_handler.setCheckState(hand_user_both, True)
                elif self.valve_status.hands == self.valve_status.USER_LH:
                    self.menu_handler.setCheckState(hand_user_left, True)
                elif self.valve_status.hands == self.valve_status.USER_RH:
                    self.menu_handler.setCheckState(hand_user_right, True)
                elif self.valve_status.hands == self.valve_status.BOTH:
                    self.menu_handler.setCheckState(hand_both, True)
                elif self.valve_status.hands == self.valve_status.RIGHT:
                    self.menu_handler.setCheckState(hand_right, True)
                elif self.valve_status.hands == self.valve_status.LEFT:
                    self.menu_handler.setCheckState(hand_left, True)
                else:
                    rospy.logwarn("Unknown Hand State Selected When Populating Menu")

            if self.valve_status.valve_type == self.valve_status.LEFTLEVER:
                hand_master = self.menu_handler.insert("Hands", callback = self.hand_cb)
                hand_right = self.menu_handler.insert("Planner Defined Right Hand", parent = hand_master, callback = self.hand_cb)
                hand_left = self.menu_handler.insert("Planner Defined Left Hand", parent = hand_master, callback = self.hand_cb)
                self.menu_handler.setCheckState(hand_right, False)
                self.menu_handler.setCheckState(hand_left, False)

                if self.valve_status.hands == self.valve_status.RIGHT:
                    self.menu_handler.setCheckState(hand_right, True)
                elif self.valve_status.hands == self.valve_status.LEFT:
                    self.menu_handler.setCheckState(hand_left, True)
                else:
                    rospy.logwarn("Unknown Hand State Selected When Populating Menu")

            if self.valve_status.valve_type == self.valve_status.RIGHTLEVER:
                hand_master = self.menu_handler.insert("Hands", callback = self.hand_cb)
                hand_left = self.menu_handler.insert("Planner Defined Left Hand", parent = hand_master, callback = self.hand_cb)
                hand_right = self.menu_handler.insert("Planner Defined Right Hand", parent = hand_master, callback = self.hand_cb)
                self.menu_handler.setCheckState(hand_left, False)
                self.menu_handler.setCheckState(hand_right, False)

                if self.valve_status.hands == self.valve_status.LEFT:
                    self.menu_handler.setCheckState(hand_left, True)
                elif self.valve_status.hands == self.valve_status.RIGHT:
                    self.menu_handler.setCheckState(hand_right, True)
                else:
                    rospy.logwarn("Unknown Hand State Selected When Populating Menu")



            #Type Options
            type_master = self.menu_handler.insert("Valve Type", callback = self.type_cb)
            type_round = self.menu_handler.insert("Round", parent = type_master, callback = self.type_cb)
            type_left = self.menu_handler.insert("Left Lever", parent=type_master, callback = self.type_cb)
            type_right = self.menu_handler.insert("Right Lever", parent=type_master, callback = self.type_cb)
            self.menu_handler.setCheckState(type_round, False)
            self.menu_handler.setCheckState(type_left, False)
            self.menu_handler.setCheckState(type_right, False)

            if self.valve_status.valve_type == self.valve_status.ROUND:
                self.menu_handler.setCheckState(type_round, True)
            elif self.valve_status.valve_type == self.valve_status.LEFTLEVER:
                self.menu_handler.setCheckState(type_left, True)
            elif self.valve_status.valve_type == self.valve_status.RIGHTLEVER:
                self.menu_handler.setCheckState(type_right, True)
            else:
                rospy.logwarn("Unknown Valve Type State Selected When Populating Menu")

            #Turning Options
            direction_master = self.menu_handler.insert("Turn Direction", callback = self.direction_cb)
            direction_ccw = self.menu_handler.insert("Turn Counter-Clockwise", parent = direction_master, callback = self.direction_cb)
            direction_cw = self.menu_handler.insert("Turn Clockwise", parent = direction_master, callback = self.direction_cb)
            self.menu_handler.setCheckState(direction_cw, False)
            self.menu_handler.setCheckState(direction_ccw, False)

            if self.valve_status.turning_direction == self.valve_status.CW:
                self.menu_handler.setCheckState(direction_cw, True)
            elif self.valve_status.turning_direction == self.valve_status.CCW:
                self.menu_handler.setCheckState(direction_ccw, True)
            else:
                rospy.logwarn("Unknown Turn Direction State Selected When Populating Menu")



    def insertTrajMenuOption(self, option):
        self.trajMenu.menuOptions.add(option)
        self.populate_valve_menu()

    def removeTrajMenuOption(self, option):
        self.trajMenu.menuOptions.remove(option)
        self.populate_valve_menu()

    def flushTrajMenu(self):
        self.trajMenu.menuOptions.clear()
        self.populate_valve_menu()



    def snap_icp_cb(self, feedback):
        print "Snapping with ICP"
        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()



    def planning_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if handle == planning_getready:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.GETREADY)

        elif handle == planning_grasp:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.GRASP)

        elif handle == planning_ungrasp:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.UNGRASP)

        elif handle == planning_turning:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.TURN)

        elif handle == planning_finish:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.FINISH)

        elif handle == planning_preview:
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.PREVIEW)

        else:
            rospy.roswarn("Planning > Unknown Plan Clicked!")



    def execute_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if handle == execute:
            print "Executing"
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.EXECUTE)
        elif handle == execute_stored:
            print "Executing Stored"
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.EXECUTE_STORED)
        else:
            rospy.roswarn("Unknown Execution Clicked!")


    def trajectory_cb(self, feedback):
        print "Trajectory Menu Option Clicked"


    def radius_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if handle == radius_increase:
            self.valve_status.radius += 0.01

        elif handle == radius_decrease:
            self.valve_status.radius += (-0.01)

        elif handle == radius_default:
            self.valve_status.radius = self.valve_status.default_radius

        else:
            rospy.roswarn("Radius > Unknown Radius Clicked!")



    def pose_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if handle == pose_reset_default:
            self.valve_status.pose_stamped = deepcopy(self.valve_status.default_pose_stamped)

        elif handle == pose_reset_session_defaul:
            self.valve_status.pose_stamped = deepcopy(self.valve_status.session_pose_stamped)

        elif handle == pose_set_default:
            self.valve_status.session_pose_stamped = deepcopy(self.valve_status.pose_stamped)

        else:
            rospy.roswarn("Pose > Unknown Pose Clicked!")



    def hand_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if self.valve_status.valve_type == self.valve_status.LEFTLEVER or\
           self.valve_status.valve_type == self.valve_status.RIGHTLEVER:

            if handle == hand_left:
                self.valve_status.hands = self.valve_status.LEFT
            elif handle == hand_right:
                self.valve_status.hands = self.valve_status.RIGHT
            else:
                rospy.roswarn("Hand > Unknown Hand Clicked!")

        if self.valve_status.valve_type == self.valve_status.ROUND:
            if handle == hand_switch:
                #TODO Make this remember the previous state for the other option
                if self.valve_status.end_effector == self.valve_status.GRIPPER:
                    self.valve_status.end_effector = self.valve_status.PEG
                    self.valve_status.hands = self.valve_status.USER_BH
                elif self.valve_status.end_effector == self.valve_status.PEG:
                    self.valve_status.end_effector = self.valve_status.GRIPPER
                    self.valve_status.hands = self.valve_status.BOTH
            elif handle == hand_user_left:
                self.valve_status.hands = self.valve_status.USER_LH
            elif handle == hand_user_right:
                self.valve_status.hands = self.valve_status.USER_RH
            elif handle == hand_user_both:
                self.valve_status.hands = self.valve_status.USER_BH
            elif handle == hand_both:
                self.valve_status.hands = self.valve_status.BOTH
            elif handle == hand_left:
                self.valve_status.hands = self.valve_status.LEFT
            elif handle == hand_right:
                self.valve_status.hands = self.valve_status.RIGHT
            else:
                rospy.roswarn("Hand > Unknown Hand Clicked!")

        self.populate_valve_menu()


    def type_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if handle == type_round:
            self.valve_status.valve_type = self.valve_status.ROUND
            self.valve_status.hands = self.valve_status.BOTH
            self.valve_status.end_effector = self.valve_status.GRIPPER

        elif handle == type_left:
            self.valve_status.valve_type = self.valve_status.LEFTLEVER
            self.valve_status.hands = self.valve_status.RIGHT
            self.valve_status.end_effector = self.valve_status.GRIPPER

        elif handle == type_right:
            self.valve_status.valve_type = self.valve_status.RIGHTLEVER
            self.valve_status.hands = self.valve_status.LEFT
            self.valve_status.end_effector = self.valve_status.GRIPPER

        else:
            rospy.roswarn("Type > Unknown Type Clicked!")

        self.populate_valve_menu()



    def direction_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if handle == direction_cw:
            self.valve_status.turning_direction = self.valve_status.CW

        elif handle == direction_ccw:
            self.valve_status.turning_direction = self.valve_status.CCW

        else:
            rospy.roswarn("Direction > Unknown Direction Clicked!")

        self.populate_valve_menu()


    def lock_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if handle == lock_master:
            if self.valve_status.lock_state == self.valve_status.UNLOCKED:
                self.valve_status.lock_state = self.valve_status.LOCKED
                self.populate_valve_menu()
            elif self.valve_status.lock_state == self.valve_status.LOCKED:
                self.valve_status.lock_state = self.valve_status.UNLOCKED
                self.populate_valve_menu()

        else:
            rospy.roswarn("Direction > Unknown Direction Clicked!")





    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.valve_status.color = "CLEAR"
            self.valve_status.pose_stamped.pose = feedback.pose
            self.flushTrajMenu()
        elif (event_type == feedback.MENU_SELECT):
            pass
        else:
            rospy.logerr("Unrecognized feedback type - " + str(feedback.event_type))

##################################
#   Peg Menu Stuff           #
##################################

    def peg_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type

        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            if (feedback.marker_name == self.left_peg.name):
                self.left_peg.poseStamped.pose = cur_pose
                self.left_peg.init = True
                self.left_peg.gripperOffset.pose = ComposePoses(InvertPose(self.valve_status.pose_stamped.pose), self.left_peg.poseStamped.pose)
            if (feedback.marker_name == self.right_peg.name):
                self.right_peg.poseStamped.pose = cur_pose
                self.right_peg.init = True
                self.right_peg.gripperOffset.pose = ComposePoses(InvertPose(self.valve_status.pose_stamped.pose), self.right_peg.poseStamped.pose)
        elif (event_type == feedback.MENU_SELECT):
            self.process_peg_menu_select(feedback)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))

##################################
#   Gripper Menu Stuff           #
##################################

    def populate_gripper_menu(self):
        self.gripper_options = ["<-- Left", "Right -->", "(+) 30 Deg", "(-) 30 Deg", "[+] Longer", "[-] Shorter"]
        self.gripper_menu_handler = MenuHandler()
        i = 1
        for menu_option in self.gripper_options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.gripper_menu_handler.insert(menu_option, callback=self.gripper_feedback_cb)
            i += 1


    def gripper_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type

        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            if (feedback.marker_name == self.left_gripper.name):

                self.left_gripper.gripperOffset.pose = ComposePoses(InvertPose(self.left_gripper.poseStamped.pose), cur_pose)

                if feedback.control_name == "translate_x":

                    if self.left_gripper.gripperOffset.pose.position.x >= 0.01:
                        self.left_gripper.length += -.02

                    if self.left_gripper.gripperOffset.pose.position.x <= -0.01:
                        self.left_gripper.length += .02

                if feedback.control_name == "translate_z":

                    if self.left_gripper.gripperOffset.pose.position.y >= 0.01:
                        self.left_gripper.circ_angle += -.05

                    if self.left_gripper.gripperOffset.pose.position.y <= -0.01:
                        self.left_gripper.circ_angle += .05

                if feedback.control_name == "translate_y":

                    if self.left_gripper.gripperOffset.pose.position.z >= 0.01:
                        self.left_gripper.angle += .04

                    if self.left_gripper.gripperOffset.pose.position.z <= -0.01:
                        self.left_gripper.angle += -.04

            if (feedback.marker_name == self.right_gripper.name):

                self.right_gripper.gripperOffset.pose = ComposePoses(InvertPose(self.right_gripper.poseStamped.pose), cur_pose)

                if feedback.control_name == "translate_x":

                    if self.right_gripper.gripperOffset.pose.position.x >= 0.01:
                        self.right_gripper.length += -.02

                    if self.right_gripper.gripperOffset.pose.position.x <= -0.01:
                        self.right_gripper.length += .02

                if feedback.control_name == "translate_z":

                    if self.right_gripper.gripperOffset.pose.position.y >= 0.01:
                        self.right_gripper.circ_angle += -.05

                    if self.right_gripper.gripperOffset.pose.position.y <= -0.01:
                        self.right_gripper.circ_angle += .05

                if feedback.control_name == "translate_y":

                    if self.right_gripper.gripperOffset.pose.position.z >= 0.01:
                        self.right_gripper.angle += .04

                    if self.right_gripper.gripperOffset.pose.position.z <= -0.01:
                        self.right_gripper.angle += -.04

        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.gripper_options[feedback.menu_entry_id - 1])
            self.process_gripper_menu_select(feedback)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))

  
    def process_gripper_menu_select(self, feedback):

        if (feedback.marker_name == self.left_gripper.name):
            # (+) 30 Deg == +.2617
            if (feedback.menu_entry_id == 1): 
                self.left_gripper.circ_angle += (-.2617)
            # (-) 30 Deg == -.2617
            elif (feedback.menu_entry_id == 2): 
                self.left_gripper.circ_angle += (+.2617)
            # (+) 30 Deg == +.2617
            elif (feedback.menu_entry_id == 3):
                self.left_gripper.angle += (-.2617 * 2)
            # (-) 30 Deg == -.2617
            elif (feedback.menu_entry_id == 4):
               self.left_gripper.angle += (+.2617 * 2)

            # [+] Longer
            elif (feedback.menu_entry_id == 5):
                self.left_gripper.length += .01
            # [-] Shorter
            elif (feedback.menu_entry_id == 6):
                self.left_gripper.length += (-.01)
            else:
                rospy.logerr("Unrecognized menu entry")

        if (feedback.marker_name == self.right_gripper.name):
            # (+) 30 Deg == +.2617
            if (feedback.menu_entry_id == 1): 
                self.right_gripper.circ_angle += (-.2617)
            # (-) 30 Deg == -.2617
            elif (feedback.menu_entry_id == 2): 
                self.right_gripper.circ_angle += (+.2617)
            # (+) 30 Deg == +.2617
            elif (feedback.menu_entry_id == 3):
                self.right_gripper.angle += (-.2617 * 2)
            # (-) 30 Deg == -.2617
            elif (feedback.menu_entry_id == 4):
               self.right_gripper.angle += (+.2617 * 2)

            # [+] Longer
            elif (feedback.menu_entry_id == 5):
                self.right_gripper.length += .01
            # [-] Shorter
            elif (feedback.menu_entry_id == 6):
                self.right_gripper.length += (-.01)
            else:
                rospy.logerr("Unrecognized menu entry")

##################################
#   Stuff For Making a peg   #
##################################

    def make_peg_imarker(self, peg):

        peg_marker = self.make_peg_marker(peg)

        #Make the Interactive Marker for each peg
        peg_imarker = InteractiveMarker()
        peg_imarker.header.frame_id = peg.poseStamped.header.frame_id
        peg_imarker.pose = peg_marker.pose
        peg_imarker.scale = peg.length * .75
        peg_imarker.name = peg.name

        # Make the default control for the marker itself
        #base_control = InteractiveMarkerControl()
        #base_control.orientation_mode = InteractiveMarkerControl.FIXED
        #base_control.always_visible = True
        ##peg_marker = self.make_peg_marker(peg)
        #base_control.markers.append(peg_marker)
        #peg_imarker.controls.append(base_control)

        # Make the menu control for the peg marker
        peg_menu_control = InteractiveMarkerControl()
        peg_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        peg_menu_control.always_visible = True
        peg_menu_control.orientation_mode = InteractiveMarkerControl.FIXED
        peg_menu_control.markers.append(peg_marker)
        peg_imarker.controls.append(peg_menu_control)

        if True == False:
            # Make the x-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "translate_x"
            new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 1.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 0.0
            peg_imarker.controls.append(new_control)

        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_y"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        peg_imarker.controls.append(new_control)

        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_z"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        peg_imarker.controls.append(new_control)

        return peg_imarker



    def make_peg_marker(self, peg):
        arrow = Marker()
        arrow.header.frame_id = peg.poseStamped.header.frame_id

        #Give it a unique ID
        arrow.ns = peg.name
        arrow.id = 0
        arrow.type = Marker.ARROW
        pose_offset = Pose()

        # Put the Arrow in the same frame as the wheel
        q1 = quaternion_about_axis(math.pi / 2, (0,1,0))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(self.valve_status.pose_stamped.pose, pose_offset)
        arrow.pose = rotated_pose

        # Rotate the arrow out of the wheel and make it so that the point is on the wheel
        q1 = quaternion_about_axis(-math.pi / 2, (0,1,0))
        pose_offset.position.z = -peg.length
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(arrow.pose, pose_offset)
        arrow.pose = rotated_pose

        # Allow the arrow to be moved around freely itself
        q1 = quaternion_about_axis(0, (1,0,0))
        pose_offset.position.x = 0
        pose_offset.position.y = peg.gripperOffset.pose.position.y
        pose_offset.position.z = peg.gripperOffset.pose.position.z
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(arrow.pose, pose_offset)
        arrow.pose = rotated_pose

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        arrow.scale.x = (peg.length)
        arrow.scale.y = (.02)
        arrow.scale.z = (.02)

        #Set the color -- be sure to set alpha to something non-zero!
        if peg.name == "right_peg":
            arrow.color.r = 1.0
            arrow.color.b = 0.0
            arrow.color.g = 0.0
        elif peg.name == "left_peg":
            arrow.color.r = 0.0
            arrow.color.b = 0.0
            arrow.color.g = 1.0
        else:
            arrow.color.r = 0.0
            arrow.color.b = 1.0
            arrow.color.g = 0.0

        arrow.color.a = 1.0
        arrow.lifetime = rospy.Duration(1)
        return arrow


##################################
#   Stuff For Making a Gripper   #
##################################

    def make_gripper_imarker(self, gripper):

        gripper_marker = self.make_gripper_marker(gripper)
        gripper.poseStamped.pose = deepcopy(gripper_marker.pose)

        #Make the Interactive Marker for each gripper
        gripper_imarker = InteractiveMarker()
        gripper_imarker.header.frame_id = gripper.poseStamped.header.frame_id
        gripper_imarker.pose = gripper_marker.pose
        gripper_imarker.scale = .13 * .75
        gripper_imarker.name = gripper.name

        # Make the menu control for the gripper marker
        gripper_menu_control = InteractiveMarkerControl()
        gripper_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_menu_control.always_visible = True
        gripper_menu_control.orientation_mode = InteractiveMarkerControl.FIXED
        gripper_menu_control.markers.append(gripper_marker)
        gripper_imarker.controls.append(gripper_menu_control)

        if True == True:
            # Make the x-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "translate_x"
            new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 1.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 0.0
            gripper_imarker.controls.append(new_control)

        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_y"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        gripper_imarker.controls.append(new_control)

        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_z"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        gripper_imarker.controls.append(new_control)

        return gripper_imarker



    def make_gripper_marker(self, gripper):
        gripper_marker = Marker()
        gripper_marker.header.frame_id = gripper.poseStamped.header.frame_id

        #Give it a unique ID
        gripper_marker.ns = gripper.name
        gripper_marker.id = 0
        gripper_marker.type = Marker.ARROW
        pose_offset = Pose()

        # Put the Arrow in the same frame as the wheel
        q1 = quaternion_about_axis(math.pi / 2, (0,1,0))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(self.valve_status.pose_stamped.pose, pose_offset)
        gripper_marker.pose = rotated_pose

        # Rotate the Arrow around the center of the wheel
        angle_rad = gripper.circ_angle
        q1 = quaternion_about_axis(angle_rad, (0,0,1))

        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper_marker.pose, pose_offset)
        gripper_marker.pose = rotated_pose

        # Rotate the grip angle while keeping the point on the wheel
        angle_rad = gripper.angle
        q1 = quaternion_about_axis(angle_rad, (0,1,0))

        pose_offset.position.x = (-1 * self.valve_status.radius * 2) + \
        (self.valve_status.radius - gripper.length * cos(angle_rad))

        pose_offset.position.z = gripper.length * sin(angle_rad)
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper_marker.pose, pose_offset)
        gripper_marker.pose = rotated_pose


        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        gripper_marker.scale.x = gripper.length
        gripper_marker.scale.y = (.02)
        gripper_marker.scale.z = (.02)

        #Set the color -- be sure to set alpha to something non-zero!
        if gripper.name == "right_gripper":
            gripper_marker.color.r = 1.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.g = 0.0
        elif gripper.name == "left_gripper":
            gripper_marker.color.r = 0.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.g = 1.0
        else:
            gripper_marker.color.r = 0.0
            gripper_marker.color.b = 1.0
            gripper_marker.color.g = 0.0

        gripper_marker.color.a = 1.0
        gripper_marker.lifetime = rospy.Duration(1)
        return gripper_marker



################################
#   Stuff For Making a Valve   #
################################



    def make_valve_imarker(self, valve_pose):
        valve_imarker = InteractiveMarker()
        valve_imarker.header.frame_id = valve_pose.header.frame_id
        valve_imarker.pose = valve_pose.pose
        valve_imarker.scale = 1.0
        valve_imarker.name = 'valve_alignment'

        # Make the default control for the marker itself
        #base_control = InteractiveMarkerControl()
        #base_control.orientation_mode = InteractiveMarkerControl.FIXED
        #base_control.always_visible = True
        valve_marker = self.make_valve_marker(valve_pose)
        #base_control.markers.append(valve_marker)
        #valve_imarker.controls.append(base_control)

        # Make the menu control
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.markers.append(valve_marker)
        valve_imarker.controls.append(new_control)

        if self.valve_status.lock_state == self.valve_status.UNLOCKED:

            # Make the x-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "translate_x"
            new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 1.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 0.0
            valve_imarker.controls.append(new_control)

            # Make the y-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "translate_y"
            new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 0.0
            new_control.orientation.y = 1.0
            new_control.orientation.z = 0.0
            valve_imarker.controls.append(new_control)

            # Make the z-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "translate_z"
            new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 0.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 1.0
            valve_imarker.controls.append(new_control)

            # Make the x-axis rotation control
            new_control = InteractiveMarkerControl()
            new_control.name = "rotate_x"
            new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 1.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 0.0
            valve_imarker.controls.append(new_control)

            # Make the y-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "rotate_y"
            new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 0.0
            new_control.orientation.y = 1.0
            new_control.orientation.z = 0.0
            valve_imarker.controls.append(new_control)

            # Make the z-axis control
            new_control = InteractiveMarkerControl()
            new_control.name = "rotate_z"
            new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            new_control.always_visible = False
            new_control.orientation_mode = InteractiveMarkerControl.INHERIT
            new_control.orientation.w = 1.0
            new_control.orientation.x = 0.0
            new_control.orientation.y = 0.0
            new_control.orientation.z = 1.0
            valve_imarker.controls.append(new_control)

        return valve_imarker

    def make_valve_marker(self, valve_pose):
        valve = Marker()
        valve.header.frame_id = valve_pose.header.frame_id
        #Give it a unique ID
        valve.ns = "valve_alignment"
        valve.id = 1

        #Give the valve a type
        if (self.valve_status.valve_type == self.valve_status.LEFTLEVER):
            valve.type = Marker.CUBE
            pose_offset = Pose()
            pose_offset.position.y = -(self.valve_status.radius /2.0)
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(valve_pose.pose, pose_offset)
            valve.pose = rotated_pose
            #Set the scale of the marker -- 1x1x1 here means 1m on a side
            valve.scale.x = 0.05
            valve.scale.y = self.valve_status.radius
            valve.scale.z = 0.05

        elif (self.valve_status.valve_type == self.valve_status.RIGHTLEVER):
            valve.type = Marker.CUBE
            pose_offset = Pose()
            pose_offset.position.y = (self.valve_status.radius /2.0)
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(valve_pose.pose, pose_offset)
            valve.pose = rotated_pose
            #Set the scale of the marker -- 1x1x1 here means 1m on a side
            valve.scale.x = self.valve_status.default_thickness
            valve.scale.y = self.valve_status.radius
            valve.scale.z = 0.05

        else:
            valve.type = Marker.CYLINDER
            pose_offset = Pose()
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(valve_pose.pose, pose_offset)
            valve.pose = rotated_pose

            #Set the scale of the valve -- 1x1x1 here means 1m on a side
            valve.scale.x = (self.valve_status.radius * 2.0)
            valve.scale.y = (self.valve_status.radius * 2.0)
            valve.scale.z = self.valve_status.default_thickness

        #Set the color -- be sure to set alpha to something non-zero!

        if self.valve_status.color == "CLEAR":
            valve.color.r = 1.0
            valve.color.b = 1.0
            valve.color.g = 0.0
            valve.color.a = 0.5
        elif self.valve_status.color == "SOLID":
            valve.color.r = 1.0
            valve.color.b = 1.0
            valve.color.g = 0.0
            valve.color.a = 1.0
        elif self.valve_status.color == "GREEN":
            valve.color.r = 0.0
            valve.color.b = 0.0
            valve.color.g = 1.0
            valve.color.a = 1.0
        elif self.valve_status.color == "RED":
            valve.color.r = 1.0
            valve.color.b = 0.0
            valve.color.g = 0.0
            valve.color.a = 1.0

        valve.lifetime = rospy.Duration(1)
        return valve





if __name__ == '__main__':
    rospy.init_node("valve_localization_gripper_no_planner")
    path = subprocess.check_output("rospack find valve_localization", shell=True)
    path = path.strip("\n") + "/data"
    ims_namespace = rospy.get_param("~ims_namespace", "valve_localizer")
    data_dir = rospy.get_param("~data_dir", path)
    planner_service = rospy.get_param("~planner_service", "valve_planner/drchubo_planner/PlanningQuery")
    execution_service = rospy.get_param("~execution_service", "valve_planner/drchubo_planner/ExecutionQuery")
    ValveLocalizer(ims_namespace, data_dir, planner_service, execution_service)
