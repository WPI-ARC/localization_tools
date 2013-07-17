#!/usr/bin/python

#   Calder Phillips-Grafflin -- ARC Lab
#
#   Displays interactive markers of valve-like objects and
#   allows the user to align the objects against the world
#   represented by the robot's sensors.
#

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
from transformation_helper import *
from icp_server_msgs.msg import *
from icp_server_msgs.srv import *

class ValveStatus:

    UNSNAPPED = 0
    SNAPPED = 1
    SNAPPEDPLUS = 2
    LEFT = 1
    RIGHT = 2
    BOTH = 3
    ROUND = 0
    LEFTLEVER = 1
    RIGHTLEVER = 2
    READY = 0
    PLANNING = 1
    PLANNED = 2
    EXECUTING = 3
    EXECUTED = 4
    ERROR = -1

    def __init__(self):
        self.default_radius = 0.1
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

class ValveLocalizer:

    def __init__(self, marker_namespace, data_dir):
        self.marker_namespace = marker_namespace
        self.populate_menu()
        self.status = ValveStatus()
        #Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)
        self.update()
        rospy.spin()

    def update(self):
        for marker_name in self.server.marker_contexts.keys():
            self.server.erase(marker_name)
        alignment_marker = self.make_alignment_imarker(self.status.pose_stamped)
        status_marker = self.make_status_imarker(self.status.pose_stamped)
        self.server.insert(alignment_marker, self.alignment_feedback_cb)
        self.server.insert(status_marker, self.menu_feedback_cb)
        self.menu_handler.apply(self.server, "valve_status")
        self.server.applyChanges()

    def populate_menu(self):
        self.options = ["Snap with ICP", "Export valve pose", "Increase radius by 0.5cm", "Decrease radius by 0.5cm", "Reset to default radius", "Reset to default pose", "Reset to session default pose", "Set session default pose", "Use LEFT hand", "Use RIGHT hand", "Use BOTH hands", "Set valve type to ROUND", "Set valve type to LEFT LEVER", "Set valve type to RIGHT LEVER"]
        self.menu_handler = MenuHandler()
        i = 1
        for menu_option in self.options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.menu_handler.insert(menu_option, callback=self.menu_feedback_cb)
            i += 1

    def process_menu_select(self, menu_entry_id):
        if (menu_entry_id == 1):
            print "Snapping with ICP!"
        elif (menu_entry_id == 2):
            print "Exporting valve pose to planner"
        elif (menu_entry_id == 3):
            self.status.radius += 0.05
        elif (menu_entry_id == 4):
            self.status.radius += (-0.05)
        elif (menu_entry_id == 5):
            self.status.radius = self.status.default_radius
        elif (menu_entry_id == 6):
            self.status.pose_stamped = deepcopy(self.status.default_pose_stamped)
        elif (menu_entry_id == 7):
            self.status.pose_stamped = deepcopy(self.status.session_pose_stamped)
        elif (menu_entry_id == 8):
            self.status.session_pose_stamped = deepcopy(self.status.pose_stamped)
        elif (menu_entry_id == 9):
            self.status.hands = self.status.LEFT
        elif (menu_entry_id == 10):
            self.status.hands = self.status.RIGHT
        elif (menu_entry_id == 11):
            self.status.hands = self.status.BOTH
        elif (menu_entry_id == 12):
            self.status.valve_type = self.status.ROUND
        elif (menu_entry_id == 13):
            self.status.valve_type = self.status.LEFTLEVER
        elif (menu_entry_id == 14):
            self.status.valve_type = self.status.RIGHTLEVER
        else:
            rospy.logerr("Unrecognized menu entry")

    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.POSE_UPDATE):
            self.status.pose_stamped.pose = feedback.pose
        elif (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        else:
            rospy.logerr("ALIGNMENT Unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def menu_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            pass
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.options[feedback.menu_entry_id - 1])
            self.process_menu_select(feedback.menu_entry_id)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))
        self.update()

    def make_status_imarker(self, marker_pose):
        status_pose = PoseStamped()
        status_pose.header.frame_id = marker_pose.header.frame_id
        status_pose.pose.position.z = 1.0
        status_pose.pose.orientation.w = 1.0
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = status_pose.pose
        new_marker.scale = 1.0
        new_marker.name = 'valve_status'
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        display_marker = self.make_status_marker(status_pose)
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
        return new_marker

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
        # Make the x-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_x"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = True
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
        new_control.always_visible = True
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
        new_control.always_visible = True
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
        new_control.always_visible = True
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
        new_control.always_visible = True
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
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)
        return new_marker

    def make_status_marker(self, marker_pose):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        marker.pose = marker_pose.pose
        #Give it a unique ID
        marker.ns = "valve_localizer"
        marker.id = 1
        #Give the marker a type
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = self.make_status_text()
        marker.scale.z = 0.25
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def make_status_text(self):
        status_text = "Valve localization:\nRadius: " + str(self.status.radius) + "\nICP status: "
        if (self.status.icp_status == self.status.SNAPPED):
            status_text += "snapped"
        elif (self.status.icp_status == self.status.SNAPPEDPLUS):
            status_text += "snapped+"
        elif (self.status.icp_status == self.status.UNSNAPPED):
            status_text += "unsnapped"
        else:
            status_text += "unknown (error!)"
        return status_text

    def make_valve_marker(self, marker_pose):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = "valve_status"
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
            marker.scale.x = 0.05
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
            marker.scale.x = self.status.radius
            marker.scale.y = self.status.radius
            marker.scale.z = 0.05
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

if __name__ == '__main__':
    rospy.init_node("rviz_people_tracker")
    path = subprocess.check_output("rospack find valve_localization", shell=True)
    path = path.strip("\n") + "/data"
    ims_namespace = rospy.get_param("~ims_namespace", "valve_localizer")
    data_dir = rospy.get_param("~data_dir", path)
    ValveLocalizer(ims_namespace, data_dir)
