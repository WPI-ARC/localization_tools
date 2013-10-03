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


# [ ] Make each arrow remember it's own pose
# [ ] Add a menu option to send the left and right grippers out
# [ ] Add option to show / display arrows on the wheel
# [ ] Add a call to enable / disable the entire plugin
# [ ] (?) Turn off the control for the valve when moving arrows?
# [ ] Confirm arrow angle movement amount
# [X] Add Option to change the length of the arrow (approach length)
# [ ] Add model of the hands to the end of each arrow

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
    FINISH = "END"
    PREVIEW = "PREVIEW"
    EXECUTE = "EXECUTE"

    def __init__(self):
        self.default_radius = 0.1
        self.default_thickness = 0.02
        self.radius = self.default_radius
        self.default_pose_stamped = PoseStamped()
        self.default_pose_stamped.header.frame_id = "/Body_TSY"
        self.default_pose_stamped.pose.position.x = 1.0
        self.default_pose_stamped.pose.position.y = 0.0
        self.default_pose_stamped.pose.position.z = 0.0
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

# Left RED, right GREEN, none BLUE
class GripperStatus:

    def __init__(self):
        self.max_markers = 2
        self.current_left = 0
        self.current_right = 1
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

        #Setup The Valve and Valve Marker
        self.populate_menu()
        self.status = ValveStatus()

        #Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)

        #Setup the connection to the planning software
        rospy.loginfo("NOT Connecting to planner...")
        #self.planner_client = rospy.ServiceProxy(planner_service, PlanTurning)
        #self.planner_client.wait_for_service()
        #self.execution_client = rospy.ServiceProxy(execution_service, ExecuteTurning)
        #self.execution_client.wait_for_service()
        rospy.loginfo("...NOT Connected to planner")

        # Setup the Grippers / Arrows
        self.gripper_status = GripperStatus()
        self.populate_gripper_menu()

        for x in range (0, self.gripper_status.max_markers):
            self.gripper_status.name.append("gripper_alignment" + str(x))
            self.gripper_status.angle.append(0.0)
            self.gripper_status.length.append(.1)
            self.gripper_status.visible.append(False)

            if x == 0: self.gripper_status.circ_angle.append(math.pi / 2)
            if x == 1: self.gripper_status.circ_angle.append((3 * math.pi) / 2)

        # Setup the Refresh Rate of the Marker
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



    def show_tool(self, data):
        self.status.visible = data.data

        if data.data == False:
            for x in range (0, self.gripper_status.max_markers):
                self.gripper_status.visible[x] = data.data


    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()

        if self.status.visible == True:
            alignment_marker = self.make_alignment_imarker(self.status.pose_stamped)
            self.server.insert(alignment_marker, self.alignment_feedback_cb)
            self.menu_handler.apply(self.server, "valve_alignment")

        for x in range (0, self.gripper_status.max_markers):
            if self.gripper_status.visible[x]:
                gripper_marker_2 = self.make_gripper_imarker(self.status.pose_stamped, x)
                self.server.insert(gripper_marker_2, self.gripper_feedback_cb)
                self.gripper_menu_handler.apply(self.server, "gripper_alignment" + str(x))

        self.server.applyChanges()




    def call_icp(self):
        if (self.status.last_pointcloud == None):
            rospy.logerr("No pointcloud available for using ICP")
            return
        req = RunICPRequest()
        # Set the target pointcloud
        req.Request.TargetPointCloud = self.status.last_pointcloud
        # Set the origin of the bounding box
        req.Request.UseBoundingVolume = True
        req.Request.BoundingVolumeOrigin = valve_in_sensor_frame_transform
        # Set the bounding box size
        req.Request.BoundingVolumeDimensions.x = self.status.radius * 3.0
        req.Request.BoundingVolumeDimensions.y = self.status.radius * 3.0
        req.Request.BoundingVolumeDimensions.z = self.status.radius * 3.0
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
            self.status.pose_stamped = ComposePoses(self.status.pose_stamped.pose,pose_adjustment)




    def call_planner(self, stage):
        self.status.planning_status = stage
        req = PlanTurningRequest()
        req.Request.ValvePose = self.status.pose_stamped
        req.Request.ValveSize = self.status.radius
        req.Request.Hands = self.status.hands
        req.Request.ValveType = self.status.valve_type
        req.Request.Direction = self.status.turning_direction
        req.Request.TaskStage = self.status.planning_status

        req.useLeft = True
        req.useRight = True
        req.leftPoseStamped = self.gripper_status.leftPoseStamped
        req.rightPoseStamped = self.gripper_status.rightPoseStamped

        res = None
        try:
            rospy.loginfo("Sending call to planner with request: " + str(req))
            res = self.planner_client.call(req)
            error_code = res.Response.ErrorCode
            labels = res.Response.Labels
            rospy.loginfo("Planner returned with error code: " + error_code + " | Labels: " + str(labels))
            self.status.operating_status = self.status.PLANNED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the planning server running?")
            self.status.operating_status = self.status.ERROR




    def call_execute(self, mode):
        req = ExecuteTurningRequest()
        req.Identifier = mode
        res = None
        try:
            rospy.loginfo("Sending execute call")
            self.status.operating_status = self.status.EXECUTING
            res = self.execution_client.call(req)
            error_code = res.ErrorCode
            rospy.loginfo("Trajectories executed with error code: " + error_code)
            self.status.operating_status = self.status.EXECUTED
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the execution server running?")
            self.status.operating_status = self.status.ERROR





##################################
#   Valve Menu Stuff             #
##################################

    def populate_menu(self):
        self.options = ["Snap with ICP", "Plan GETREADY", "Plan TURNING", "Plan FINISH", "Preview plan", "Execute plan", "Increase radius by 1.0cm", "Decrease radius by 1.0cm", "Reset to default radius", "Reset to default pose", "Reset to session default pose", "Set session default pose", "Use LEFT hand", "Use RIGHT hand", "Use BOTH hands", "Set valve type to ROUND", "Set valve type to LEFT LEVER", "Set valve type to RIGHT LEVER", "Turn valve CLOCKWISE", "Turn valve COUNTERCLOCKWISE", "Make Gripper Position"]
        self.menu_handler = MenuHandler()
        i = 1
        for menu_option in self.options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.menu_handler.insert(menu_option, callback=self.alignment_feedback_cb)
            i += 1

    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.status.pose_stamped.pose = feedback.pose
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.options[feedback.menu_entry_id - 1])
            self.process_menu_select(feedback.menu_entry_id)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))

    def process_menu_select(self, menu_entry_id):
        if (menu_entry_id == 1):
            print "Snapping with ICP!"
        elif (menu_entry_id == 2):
            self.call_planner(self.status.GETREADY)
        elif (menu_entry_id == 3):
            self.call_planner(self.status.TURN)
        elif (menu_entry_id == 4):
            self.call_planner(self.status.FINISH)
        elif (menu_entry_id == 5):
            self.call_execute(self.status.PREVIEW)
        elif (menu_entry_id == 6):
            self.call_execute(self.status.EXECUTE)
        elif (menu_entry_id ==7):
            self.status.radius += 0.01
        elif (menu_entry_id == 8):
            self.status.radius += (-0.01)
        elif (menu_entry_id == 9):
            self.status.radius = self.status.default_radius
        elif (menu_entry_id == 10):
            self.status.pose_stamped = deepcopy(self.status.default_pose_stamped)
        elif (menu_entry_id == 11):
            self.status.pose_stamped = deepcopy(self.status.session_pose_stamped)
        elif (menu_entry_id == 12):
            self.status.session_pose_stamped = deepcopy(self.status.pose_stamped)
        elif (menu_entry_id == 13):
            self.status.hands = self.status.LEFT
        elif (menu_entry_id == 14):
            self.status.hands = self.status.RIGHT
        elif (menu_entry_id == 15):
            self.status.hands = self.status.BOTH
        elif (menu_entry_id == 16):
            self.status.valve_type = self.status.ROUND
        elif (menu_entry_id == 17):
            self.status.valve_type = self.status.LEFTLEVER
        elif (menu_entry_id == 18):
            self.status.valve_type = self.status.RIGHTLEVER
        elif (menu_entry_id == 19):
            self.status.turning_direction = self.status.CW
        elif (menu_entry_id == 20):
            self.status.turning_direction = self.status.CCW
        elif (menu_entry_id == 21):
            for x in range (0, self.gripper_status.max_markers):
                self.gripper_status.visible[x] = True
        else:
            rospy.logerr("Unrecognized menu entry")




##################################
#   Gripper Menu Stuff           #
##################################

    def populate_gripper_menu(self):
        self.gripper_options = ["Right Hand", "Left Hand", "<-- Left", "Right -->", "(+) 30 Deg", "(-) 30 Deg", "[+] Longer", "[-] Shorter"]
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
        if (feedback.menu_entry_id == 1):
            #self.gripper_status.current_right = index
            rospy.logwarn("I can't do that Dave.")
        #Left Hand
        elif (feedback.menu_entry_id == 2):
            #self.gripper_status.current_left = index
            rospy.logwarn("I can't do that Dave.")

        # (+) 30 Deg == +.2617
        elif (feedback.menu_entry_id == 3):
            self.gripper_status.circ_angle[index] += .2617
        # (-) 30 Deg == -.2617
        elif (feedback.menu_entry_id == 4):
            self.gripper_status.circ_angle[index] += (-.2617)

        # (+) 30 Deg == +.2617
        elif (feedback.menu_entry_id == 5):
            self.gripper_status.angle[index] += .2617 * 2
        # (-) 30 Deg == -.2617
        elif (feedback.menu_entry_id == 6):
            self.gripper_status.angle[index] += (-.2617 * 2)

        # [+] Longer
        elif (feedback.menu_entry_id == 7):
            self.gripper_status.length[index] += .01
        # [-] Shorter
        elif (feedback.menu_entry_id == 8):
            self.gripper_status.length[index] += (-.01)
        else:
            rospy.logerr("Unrecognized menu entry")





##################################
#   Stuff For Making a Gripper   #
##################################

    def make_gripper_imarker(self, marker_pose, marker_id):
        gripper_marker = InteractiveMarker()
        gripper_marker.header.frame_id = marker_pose.header.frame_id
        gripper_marker.pose = marker_pose.pose
        gripper_marker.scale = self.gripper_status.length[marker_id] * .75
        gripper_marker.name = 'gripper_alignment'+str(marker_id)

        #Make the Default Control that will be built off of
        #base_control = InteractiveMarkerControl()
        #base_control.orientation_mode = InteractiveMarkerControl.FIXED
        #base_control.always_visible = True
        gripper_display_marker = self.make_gripper_marker(marker_pose, marker_id)
        #base_control.markers.append(gripper_display_marker)
        #gripper_marker.controls.append(base_control)

        # Make the menu control for the gripper marker
        gripper_menu_control = InteractiveMarkerControl()
        gripper_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_menu_control.always_visible = True
        gripper_menu_control.orientation_mode = InteractiveMarkerControl.FIXED
        gripper_menu_control.markers.append(gripper_display_marker)
        gripper_marker.controls.append(gripper_menu_control)

        return gripper_marker



    def make_gripper_marker(self, marker_pose, marker_id):
        gripper = Marker()
        gripper.header.frame_id = marker_pose.header.frame_id

        #Give it a unique ID
        gripper.ns = "gripper_alignment" + str(marker_id)
        gripper.id = marker_id
        gripper.type = Marker.ARROW
        pose_offset = Pose()

        # Put the Arrow in the same frame as the wheel
        q1 = quaternion_about_axis(math.pi / 2, (0,1,0))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
        gripper.pose = rotated_pose

        # Rotate the Arrow around the center of the wheel

        #delta = (2 * math.pi) / self.gripper_status.max_markers
        angle_rad = self.gripper_status.circ_angle[marker_id]
        q1 = quaternion_about_axis(angle_rad, (0,0,1))

        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper.pose, pose_offset)
        gripper.pose = rotated_pose

        # Rotate the grip angle while keeping the point on the wheel

        angle_rad = self.gripper_status.angle[marker_id]

        q1 = quaternion_about_axis(angle_rad, (0,1,0))

        pose_offset.position.x = (-1 * self.status.radius * 2) + (self.status.radius - self.gripper_status.length[marker_id] * cos(angle_rad))

        pose_offset.position.z = self.gripper_status.length[marker_id] * sin(angle_rad)
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(gripper.pose, pose_offset)
        gripper.pose = rotated_pose

        if marker_id == self.gripper_status.current_left:
            self.gripper_status.leftPoseStamped.header.frame_id = self.status.default_pose_stamped.header.frame_id
            self.gripper_status.leftPoseStamped.pose = gripper.pose

        if marker_id == self.gripper_status.current_right:
            self.gripper_status.rightPoseStamped.header.frame_id = self.status.default_pose_stamped.header.frame_id
            self.gripper_status.rightPoseStamped.pose = gripper.pose

        ###################################################

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        gripper.scale.x = (self.gripper_status.length[marker_id])
        gripper.scale.y = (.02)
        gripper.scale.z = (.02)

        #Set the color -- be sure to set alpha to something non-zero!

        if self.gripper_status.current_left == marker_id:
            gripper.color.r = 1.0
            gripper.color.b = 0.0
            gripper.color.g = 0.0
        elif self.gripper_status.current_right == marker_id:
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



    def make_alignment_imarker(self, marker_pose):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 1.0
        new_marker.name = 'valve_alignment'
        # Make the default control for the marker itself
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.always_visible = True
        display_marker = self.make_valve_marker(marker_pose)
        base_control.markers.append(display_marker)
        new_marker.controls.append(base_control)
        # Make the menu control
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
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
        new_marker.controls.append(new_control)
        return new_marker

    def make_valve_marker(self, marker_pose):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = "valve_alignment"
        marker.id = 1
        #Give the marker a type
        if (self.status.valve_type == self.status.LEFTLEVER):
            marker.type = Marker.CUBE
            pose_offset = Pose()
            pose_offset.position.y = -(self.status.radius /2.0)
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
            marker.pose = rotated_pose
            #Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.05
            marker.scale.y = self.status.radius
            marker.scale.z = 0.05
        elif (self.status.valve_type == self.status.RIGHTLEVER):
            marker.type = Marker.CUBE
            pose_offset = Pose()
            pose_offset.position.y = (self.status.radius /2.0)
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
            marker.pose = rotated_pose
            #Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = self.status.default_thickness
            marker.scale.y = self.status.radius
            marker.scale.z = 0.05
        else:
            marker.type = Marker.CYLINDER
            pose_offset = Pose()
            q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
            pose_offset.orientation.x = q1[0]
            pose_offset.orientation.y = q1[1]
            pose_offset.orientation.z = q1[2]
            pose_offset.orientation.w = q1[3]
            rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
            marker.pose = rotated_pose
            #Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = (self.status.radius * 2.0)
            marker.scale.y = (self.status.radius * 2.0)
            marker.scale.z = self.status.default_thickness
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.25
        marker.lifetime = rospy.Duration(1)
        return marker





if __name__ == '__main__':
    rospy.init_node("valve_localization_gripper_no_planner")
    path = subprocess.check_output("rospack find valve_localization", shell=True)
    path = path.strip("\n") + "/data"
    ims_namespace = rospy.get_param("~ims_namespace", "valve_localizer")
    data_dir = rospy.get_param("~data_dir", path)
    planner_service = rospy.get_param("~planner_service", "valve_planner/drchubo_planner/PlanningQuery")
    execution_service = rospy.get_param("~execution_service", "valve_planner/drchubo_planner/ExecutionQuery")
    ValveLocalizer(ims_namespace, data_dir, planner_service, execution_service)
