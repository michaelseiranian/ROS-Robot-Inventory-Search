#!/usr/bin/env python3
import rospy
import actionlib
import smach_ros
from second_coursework.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult
from second_coursework.srv import GetRoomCoordResponse, GetRoomCoord, GetRoomCoordRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
from smach_ros import SimpleActionState
from yolo import Yolo


class SearchServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('search', SearchAction, self.execute, False)
        self.feedback = SearchFeedback()
        self.result = SearchResult()
        self.server.start()

    def execute(self, goal1: SearchGoal):
        room = goal1.room
        rospy.wait_for_service('return_coord')

        def get_goal_cb(userdata, goal):
            getroomcoord = rospy.ServiceProxy('return_coord', GetRoomCoord)
            service_request = GetRoomCoordRequest()
            service_request.room = room
            response: GetRoomCoordResponse = getroomcoord(service_request)
            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.get_rostime()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = response.coordinate.x
            goal.target_pose.pose.position.y = response.coordinate.y
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            return goal

        @smach.cb_interface(input_keys=['feedback'], outcomes=['cake_found', 'feedback_written'])
        def write_feedback(userdata):
            r = rospy.Rate(5)
            while not rospy.is_shutdown():
                self.feedback = userdata.feedback
                self.server.publish_feedback(self.feedback)
                if "cake" in self.feedback.object_feedback:
                    return 'cake_found'
                r.sleep()
                return 'feedback_written'

        def child_term_cb(outcome_map):
            if outcome_map['SUB2'] == 'cake_found':
                return True
            return False

        cc = smach.Concurrence(outcomes=['finish'], default_outcome='finish', child_termination_cb=child_term_cb)
        with cc:
            sm_sub1 = smach.StateMachine(outcomes=['stop_moving'])
            with sm_sub1:
                smach.StateMachine.add('MOVE_BASE',
                                       SimpleActionState('/move_base',
                                                         MoveBaseAction,
                                                         goal_cb=get_goal_cb),
                                       transitions={'succeeded': 'MOVE_BASE',
                                                    'preempted': 'stop_moving',
                                                    'aborted': 'stop_moving'})
            sm_sub2 = smach.StateMachine(outcomes=['cake_found'])
            with sm_sub2:
                smach.StateMachine.add('YOLO', Yolo(),
                                       transitions={'cake_found': 'cake_found',
                                                    'image_processed': 'WRITE_FEEDBACK'})
                smach.StateMachine.add('WRITE_FEEDBACK', smach.CBState(write_feedback),
                                       transitions={'cake_found': 'cake_found',
                                                    'feedback_written': 'YOLO'})
            smach.Concurrence.add('SUB1', sm_sub1)
            smach.Concurrence.add('SUB2', sm_sub2)

        sis = smach_ros.IntrospectionServer('server_name', cc, '/CC_ROOT')
        sis.start()

        cc.execute()
        self.result.object_result = self.feedback.object_feedback
        self.result.amount_result = self.feedback.amount_feedback
        self.result.time = rospy.Time.now()
        self.server.set_succeeded(result=self.result)


if __name__ == "__main__":
    rospy.init_node('main_node')
    SearchServer()
    rospy.spin()
