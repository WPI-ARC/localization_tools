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
    LEFT = 'LH'
    RIGHT = 'RH'
    BOTH = 'BH'
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
        self.hands = self.BOTH
        self.valve_type = self.ROUND
        self.operating_status = self.READY
        self.turning_direction = self.CW
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



# Left RED, right GREEN, none BLUE
class GripperStatus:

    def __init__(self):
        self.max_markers = 2
        self.current_left = 1
        self.current_right = 0
        self.leftPoseStamped = PoseStamped()
        self.rightPoseStamped = PoseStamped()

        self.name = []
        self.circ_angle = []
        self.angle = []
        self.length = []
        self.visible = []





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
        self.gripper_status = GripperStatus()
        self.populate_gripper_menu()

        for x in range (0, self.gripper_status.max_markers):
            self.gripper_status.name.append("gripper_alignment" + str(x))
            self.gripper_status.angle.append(0.0)
            self.gripper_status.length.append(.13)
            self.gripper_status.visible.append(True)

            if x == 0: self.gripper_status.circ_angle.append((3 * math.pi) / 2)
            if x == 1: self.gripper_status.circ_angle.append(math.pi / 2)

        # Setup the Refresh Rate of the Marker
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



    def show_tool(self, data):
        self.valve_status.visible = data.data

        for x in range (0, self.gripper_status.max_markers):
            self.gripper_status.visible[x] = data.data


    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()

        if self.valve_status.visible == True:
            alignment_marker = self.make_valve_imarker(self.valve_status.pose_stamped)
            self.server.insert(alignment_marker, self.alignment_feedback_cb)
            self.menu_handler.apply(self.server, "valve_alignment")

        for x in range (0, self.gripper_status.max_markers):
            if self.gripper_status.visible[x]:
                gripper_marker_2 = self.make_gripper_imarker(self.valve_status.pose_stamped, x)
                self.server.insert(gripper_marker_2, self.gripper_feedback_cb)
                self.gripper_menu_handler.apply(self.server, "gripper_alignment" + str(x))

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
        req.Request.Hands = self.valve_status.hands
        req.Request.ValveType = self.valve_status.valve_type
        req.Request.Direction = self.valve_status.turning_direction
        req.Request.TaskStage = self.valve_status.planning_status

        req.Request.useLeft = True
        req.Request.useRight = True
        req.Request.LeftPose = self.gripper_status.leftPoseStamped
        req.Request.RightPose = self.gripper_status.rightPoseStamped
        req.Request.LeftLength = self.gripper_status.length[self.gripper_status.current_right]
        req.Request.RightLength = self.gripper_status.length[self.gripper_status.current_left]

        res = None
        try:
            rospy.loginfo("Sending call to planner with request: " + str(req))
            res = self.planner_client.call(req)
            error_code = res.Response.ErrorCode
            labels = res.Response.Labels

            if error_code == self.trajMenu.NOERROR:
                pass
            elif error_code == self.trajMenu.SAVE:
                self.insertTrajMenuOption(labels)
            elif error_code == self.trajMenu.FLUSH:
                self.removeTrajMenuOption(labels)
            else:
                rospy.loginfo("Planner returned unknown error code: " + error_code + " | Labels: " + str(labels))

            self.valve_status.operating_status = self.valve_status.PLANNED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the planning server running?")
            self.valve_status.operating_status = self.valve_status.ERROR




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
                pass
            elif error_code == self.trajMenu.SAVE:
                self.insertTrajMenuOption(labels)
            elif error_code == self.trajMenu.FLUSH:
                self.removeTrajMenuOption(labels)
            else:
                rospy.loginfo("Trajectories executed with error code: " + error_code)

            self.valve_status.operating_status = self.valve_status.EXECUTED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the execution server running?")
            self.valve_status.operating_status = self.valve_status.ERROR





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
        global hand_master, hand_left, hand_right, hand_both
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
            hand_master = self.menu_handler.insert("Hand", callback = self.hand_cb)
            hand_left = self.menu_handler.insert("Left Hand", parent = hand_master, callback = self.hand_cb)
            hand_right = self.menu_handler.insert("Right Hand", parent = hand_master, callback = self.hand_cb)
            hand_both = self.menu_handler.insert("Both Hands", parent = hand_master, callback = self.hand_cb)

            #Type Options
            type_master = self.menu_handler.insert("Valve Type", callback = self.type_cb)
            type_round = self.menu_handler.insert("Round", parent = type_master, callback = self.type_cb)
            type_left = self.menu_handler.insert("Left Lever", parent=type_master, callback = self.type_cb)
            type_right = self.menu_handler.insert("Right Lever", parent=type_master, callback = self.type_cb)

            #Turning Options
            direction_master = self.menu_handler.insert("Turn Direction", callback = self.direction_cb)
            direction_cw = self.menu_handler.insert("Turn Clockwise", parent = direction_master, callback = self.direction_cb)
            direction_ccw = self.menu_handler.insert("Turn Counter-Clockwise", parent = direction_master, callback = self.direction_cb)



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
            self.valve_status.color = "GREEN"

        elif handle == planning_grasp:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.GRASP)
            self.valve_status.color = "GREEN"

        elif handle == planning_ungrasp:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.UNGRASP)
            self.valve_status.color = "GREEN"

        elif handle == planning_turning:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.TURN)
            self.valve_status.color = "GREEN"

        elif handle == planning_finish:
            self.valve_status.color = "SOLID"
            self.call_planner(self.valve_status.FINISH)
            self.valve_status.color = "GREEN"

        elif handle == planning_preview:
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.PREVIEW)
            self.valve_status.color = "GREEN"

        else:
            rospy.roswarn("Planning > Unknown Plan Clicked!")



    def execute_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if handle == execute:
            print "Executing"
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.EXECUTE)
            self.valve_status.color = "GREEN"
        elif handle == execute_stored:
            print "Executing Stored"
            self.valve_status.color = "SOLID"
            self.call_execute(self.valve_status.EXECUTE_STORED)
            self.valve_status.color = "GREEN"
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

        if handle == hand_left:
            self.valve_status.hands = self.valve_status.LEFT

        elif handle == hand_right:
            self.valve_status.hands = self.valve_status.RIGHT

        elif handle == hand_both:
            self.valve_status.hands = self.valve_status.BOTH

        else:
            rospy.roswarn("Hand > Unknown Hand Clicked!")



    def type_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        self.valve_status.color = "CLEAR"
        self.flushTrajMenu()

        if handle == type_round:
            self.valve_status.valve_type = self.valve_status.ROUND

        elif handle == type_left:
            self.valve_status.valve_type = self.valve_status.LEFTLEVER

        elif handle == type_right:
            self.valve_status.valve_type = self.valve_status.RIGHTLEVER

        else:
            rospy.roswarn("Type > Unknown Type Clicked!")



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
        index = self.gripper_status.name.index(feedback.marker_name)

        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            pass
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.gripper_options[feedback.menu_entry_id - 1])
            self.process_gripper_menu_select(feedback)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))

  
    def process_gripper_menu_select(self, feedback):
        index = self.gripper_status.name.index(feedback.marker_name)

        #Right Hand
        #if (feedback.menu_entry_id == 1):
        #    #self.gripper_status.current_right = index
        #    rospy.logwarn("I can't do that Dave.")
        #Left Hand
        #elif (feedback.menu_entry_id == 2):
        #    #self.gripper_status.current_left = index
        #    rospy.logwarn("I can't do that Dave.")

        # (+) 30 Deg == +.2617
        if (feedback.menu_entry_id == 1):
            self.gripper_status.circ_angle[index] += (-.2617)
        # (-) 30 Deg == -.2617
        elif (feedback.menu_entry_id == 2):
            self.gripper_status.circ_angle[index] += (+.2617)

        # (+) 30 Deg == +.2617
        elif (feedback.menu_entry_id == 3):
            self.gripper_status.angle[index] += (-.2617 * 2)
        # (-) 30 Deg == -.2617
        elif (feedback.menu_entry_id == 4):
            self.gripper_status.angle[index] += (+.2617 * 2)

        # [+] Longer
        elif (feedback.menu_entry_id == 5):
            self.gripper_status.length[index] += .01
        # [-] Shorter
        elif (feedback.menu_entry_id == 6):
            self.gripper_status.length[index] += (-.01)
        else:
            rospy.logerr("Unrecognized menu entry")





##################################
#   Stuff For Making a Gripper   #
##################################

    def make_gripper_imarker(self, gripper_pose, gripper_id):

        #Make the Interactive Marker for each gripper
        gripper_imarker = InteractiveMarker()
        gripper_imarker.header.frame_id = gripper_pose.header.frame_id
        gripper_imarker.pose = gripper_pose.pose
        gripper_imarker.scale = self.gripper_status.length[gripper_id] * .75
        gripper_imarker.name = 'gripper_alignment'+str(gripper_id)

        
        gripper_marker = self.make_gripper_marker(gripper_pose, gripper_id)

        # Make the menu control for the gripper marker
        gripper_menu_control = InteractiveMarkerControl()
        gripper_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_menu_control.always_visible = True
        gripper_menu_control.orientation_mode = InteractiveMarkerControl.FIXED
        gripper_menu_control.markers.append(gripper_marker)
        gripper_imarker.controls.append(gripper_menu_control)

        return gripper_imarker



    def make_gripper_marker(self, gripper_pose, gripper_id):
        gripper = Marker()
        gripper.header.frame_id = gripper_pose.header.frame_id

        #Give it a unique ID
        gripper.ns = "gripper_alignment" + str(gripper_id)
        gripper.id = gripper_id
        gripper.type = Marker.ARROW
        pose_offset = Pose()

        # Put the Arrow in the same frame as the wheel
        q1 = quaternion_about_axis(math.pi / 2, (0,1,0))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper_pose.pose, pose_offset)
        gripper.pose = rotated_pose

        # Rotate the Arrow around the center of the wheel
        angle_rad = self.gripper_status.circ_angle[gripper_id]
        q1 = quaternion_about_axis(angle_rad, (0,0,1))

        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper.pose, pose_offset)
        gripper.pose = rotated_pose

        # Rotate the grip angle while keeping the point on the wheel

        angle_rad = self.gripper_status.angle[gripper_id]

        q1 = quaternion_about_axis(angle_rad, (0,1,0))

        pose_offset.position.x = (-1 * self.valve_status.radius * 2) + \
        (self.valve_status.radius - self.gripper_status.length[gripper_id] * cos(angle_rad))

        pose_offset.position.z = self.gripper_status.length[gripper_id] * sin(angle_rad)
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper.pose, pose_offset)
        gripper.pose = rotated_pose

        #Create the messages for the left and right that can be sent out
        if gripper_id == self.gripper_status.current_left:
            self.gripper_status.leftPoseStamped.header.frame_id = \
                          self.valve_status.default_pose_stamped.header.frame_id
            self.gripper_status.leftPoseStamped.pose = gripper.pose

        if gripper_id == self.gripper_status.current_right:
            self.gripper_status.rightPoseStamped.header.frame_id = \
                          self.valve_status.default_pose_stamped.header.frame_id
            self.gripper_status.rightPoseStamped.pose = gripper.pose


        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        gripper.scale.x = (self.gripper_status.length[gripper_id])
        gripper.scale.y = (.02)
        gripper.scale.z = (.02)

        #Set the color -- be sure to set alpha to something non-zero!
        if self.gripper_status.current_left == gripper_id:
            gripper.color.r = 1.0
            gripper.color.b = 0.0
            gripper.color.g = 0.0
        elif self.gripper_status.current_right == gripper_id:
            gripper.color.r = 0.0
            gripper.color.b = 0.0
            gripper.color.g = 1.0
        else:
            gripper.color.r = 0.0
            gripper.color.b = 1.0
            gripper.color.g = 0.0

        gripper.color.a = 1.0
        gripper.lifetime = rospy.Duration(1)
        return gripper



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
