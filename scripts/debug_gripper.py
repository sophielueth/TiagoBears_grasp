import rospy

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from actionlib import SimpleActionClient

if __name__ == '__main__':
# init action clients to move grippers
	rospy.init_node('debug_gripper')
	ns = '/gripper_left_controller/follow_joint_trajectory'
	_gripper_client = SimpleActionClient(ns=ns, ActionSpec=FollowJointTrajectoryAction)
	_gripper_client.wait_for_server()

	_gripper_closed = [JointTrajectoryPoint(positions=[0.02, 0.02], time_from_start=rospy.Duration.from_sec(5))]
	_gripper_opened = [JointTrajectoryPoint(positions=[0.04, 0.04], time_from_start=rospy.Duration.from_sec(5))]
	_gripper_joint_names =  ['gripper_left_left_finger_joint', 'gripper_left_right_finger_joint']

	def create_goal_state(num):
		goal = JointTrajectoryPoint(positions=[num, num], time_from_start=rospy.Duration.from_sec(5))
		return [goal]

	def set_gripper(goal_state):
    # create goal for the action client for the gripper
		goal = FollowJointTrajectoryGoal()

		header = Header()
		header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.5)
		goal.trajectory = JointTrajectory(joint_names=_gripper_joint_names, points=goal_state)
		
		goal.goal_time_tolerance = rospy.Duration.from_sec(2.0)
		
		_gripper_client.wait_for_server()
		res = _gripper_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration.from_sec(10.0), preempt_timeout=rospy.Duration.from_sec(10.0))
		
		return res == 3

	def set_gripper_to(num):
		goal = create_goal_state(num)
		return set_gripper(goal)

	res = set_gripper(_gripper_opened)
	res = set_gripper(_gripper_closed)
	pass
