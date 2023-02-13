import rospy
import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

def setup():
    ## initialize move it for both arms
		moveit_commander.roscpp_initialize(sys.argv)

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander(robot_description="robot_description")

		## Instantiate a `PlanningSceneInterface`_ object; it's a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
		scene = moveit_commander.PlanningSceneInterface()

        # add table as collision object
		p = Pose(Point(x=0.5, y=0, z=0), Quaternion())
		ps = PoseStamped()
		ps.header.frame_id = robot.get_planning_frame()
		ps.pose = p
		scene.add_box('table', ps, (0.6, 0.75, 0.5))

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                               moveit_msgs.msg.DisplayTrajectory,
		                                               queue_size=20)

        # init to move torso to joint position 0.25
		torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory)


        # actually move torso to joint position 0.25 after publisher has been set up
		msg = JointTrajectory(joint_names=['torso_lift_joint'], points=[JointTrajectoryPoint(positions=[0.25], time_from_start=rospy.Duration.from_sec(1))])
        rospy.sleep(3) # TODO; decrease? 
        torso_pub.publish(msg)
