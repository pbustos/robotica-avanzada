#! /usr/bin/env python3

#import pyhton
import sys
import os
import time
import threading

#import api kortex
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def cartesian_Home_movement(base, base_cyclic):
    
    print("Starting Cartesian Home Movement ...")
    action = Base_pb2.Action()
    action.name = "Cartesian Home movement"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x =  0.35 # (meters)
    cartesian_pose.y =  0.040 # (meters)
    cartesian_pose.z =  0.40 # (meters)
    cartesian_pose.theta_x =  180.0 # (degrees)
    cartesian_pose.theta_y =  0.0 # (degrees)
    cartesian_pose.theta_z = 90.0 # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def cartesian_Especific_movement(base, base_cyclic, x, y, z, theta_x, theta_y, theta_z):
    
    print("Starting Cartesian Especific Movement ...")
    action = Base_pb2.Action()
    action.name = "Cartesian Especific movement"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x =  x # (meters)
    cartesian_pose.y =  y # (meters)
    cartesian_pose.z =  z # (meters)
    cartesian_pose.theta_x =  theta_x # (degrees)
    cartesian_pose.theta_y =  theta_y # (degrees)
    cartesian_pose.theta_z = theta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def cartesian_Relative_movement(base, base_cyclic, x, y, z, theta_x, theta_y, theta_z):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x =  feedback.base.tool_pose_x + x  # (meters)
    cartesian_pose.y =  feedback.base.tool_pose_y + y # (meters)
    cartesian_pose.z =  feedback.base.tool_pose_z + z # (meters)
    cartesian_pose.theta_x =  feedback.base.tool_pose_theta_x + theta_x # (degrees)
    cartesian_pose.theta_y =  feedback.base.tool_pose_theta_y + theta_y # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + theta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def gripper_Close_All(base):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break

    # Close the gripper with position increments
    print("Closing gripper...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = gripper_measure.finger[0].value
    finger.finger_identifier = 1
    while position < 1.0:
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        base.SendGripperCommand(gripper_command)
        position += 0.05
        time.sleep(0.05)
    
    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}\n".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break

    if((1.0 - gripper_measure.finger[0].value) < 0.2):
        return True
    else:
        return False


def gripper_Open_All(base):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break

    # Close the gripper with position increments
    print("Opening gripper...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = gripper_measure.finger[0].value
    finger.finger_identifier = 1
    while position > 0.0:
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        base.SendGripperCommand(gripper_command)
        position -= 0.05
        time.sleep(0.05)
    
    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}\n".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break

    if((gripper_measure.finger[0].value) < 0.2):
        return True
    else:
        return False

def gripper_Relative_Aperture(base, aperture):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break

    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = gripper_measure.finger[0].value
    finger.finger_identifier = 1
 
    if((aperture - position) < 0):
        print("Opening gripper...")
        while position > aperture:
            finger.value = position
            print("Going to position {:0.2f}...".format(finger.value))
            base.SendGripperCommand(gripper_command)
            position -= 0.05
            time.sleep(0.05)
    else:
        print("Closing gripper...")
        while position < aperture:
            finger.value = position
            print("Going to position {:0.2f}...".format(finger.value))
            base.SendGripperCommand(gripper_command)
            position += 0.05
            time.sleep(0.05)

    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}\n".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value >= 0 and gripper_measure.finger[0].value <= 1:
                break
        else: # Else, no finger present in answer, end loop
            break
    
    if(abs(aperture - gripper_measure.finger[0].value) < 0.2):
        return True
    else:
        return False

    