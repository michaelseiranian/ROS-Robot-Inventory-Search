#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from second_coursework.msg import SearchAction, SearchGoal

if __name__ == '__main__':
    room = 'F'
    rospy.init_node('action_client')
    client = actionlib.SimpleActionClient('search', SearchAction)
    client.wait_for_server()
    goal = SearchGoal()
    goal.room = room
    client.send_goal(goal)
    client.wait_for_result()
    status = client.get_state()
    if status == GoalStatus.SUCCEEDED:
        print("Found cake!")
    else:
        print("Cake not found, going to try next room")
