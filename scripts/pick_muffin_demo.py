#!/usr/bin/env python

import rospy
import actionlib

from muffin_msgs.msg import *
from moveit_msgs.msg import *
from shape_msgs.msg import *
from geometry_msgs.msg import *
from object_recognition_msgs.msg import *

from math import sqrt

def move_group_goal(joint_states):
	goal = MoveGroupGoal()
	goal.planning_options.planning_scene_diff.is_diff = True
	goal.planning_options.planning_scene_diff.robot_state.is_diff = True
	goal.request = MotionPlanRequest(
		group_name='arm',
		num_planning_attempts=1,
		allowed_planning_time=5.0,
		start_state=RobotState( is_diff = True ),
		goal_constraints=[Constraints(
			joint_constraints=[
				JointConstraint(
					joint_name='katana_motor1_pan_joint',
					position=joint_states[0],
					weight=1.0
				),
				JointConstraint(
					joint_name='katana_motor2_lift_joint',
					position=joint_states[1],
					weight=1.0
				),
				JointConstraint(
					joint_name='katana_motor3_lift_joint',
					position=joint_states[2],
					weight=1.0
				),
				JointConstraint(
					joint_name='katana_motor4_lift_joint',
					position=joint_states[3],
					weight=1.0
				),
				JointConstraint(
					joint_name='katana_motor5_wrist_roll_joint',
					position=joint_states[4],
					weight=1.0
				)
			]
		)]
	)
	return goal


if __name__ == '__main__':
	rospy.init_node('pick_muffin_demo')

	pspub= rospy.Publisher('/planning_scene_world', PlanningSceneWorld, queue_size= 2)
	rospy.sleep(rospy.Duration.from_sec(1.0))
	pspub.publish(PlanningSceneWorld())
	rospy.sleep(rospy.Duration.from_sec(1.0))

	copub= rospy.Publisher('/collision_object', CollisionObject, queue_size= 2)
	def tablesToPlanningScene(tables):
		for t in tables.tables:
			copub.publish( CollisionObject(
				header= t.header,
				id= "table",
				meshes= [Mesh(
					vertices= t.convex_hull,
					triangles= [ MeshTriangle(vertex_indices= [0, n, n+1]) for n in range(1, len(t.convex_hull)-1) ]
				)],
				mesh_poses= [t.pose],
				operation= CollisionObject.ADD
			) )
	rospy.Subscriber('/table', TableArray, tablesToPlanningScene)

	mgclient = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
	mgclient.wait_for_server(rospy.Duration.from_sec(2.0))

	lmclient= actionlib.SimpleActionClient('/detect_muffin', DetectMuffinAction)
	lmclient.wait_for_server(rospy.Duration.from_sec(2.0))

	piclient= actionlib.SimpleActionClient('/calvin_pick_server', PickServerAction)
	piclient.wait_for_server(rospy.Duration.from_sec(2.0))

	rospy.loginfo("move arm out of the way")
	mgclient.send_goal(move_group_goal([-1.7013, -0.1344, -1.3435, -1.9345, -0.2885]) )
	mgclient.wait_for_result(rospy.Duration.from_sec(30.0))
	ret= mgclient.get_result().error_code.val
	if ret != MoveItErrorCodes.SUCCESS and ret != MoveItErrorCodes.TIMED_OUT:
		rospy.logerr("Move Arm Away failed - %s" % mgclient.get_result().error_code.val)
		exit(1)

	rospy.loginfo("give tabletop segmentation time to cache a clean image...")
	rospy.sleep(rospy.Duration.from_sec(5.0))

	rospy.loginfo("detect muffin")
	lmclient.send_goal(DetectMuffinGoal(min_muffin_dim= .01, max_muffin_dim= .13))
	lmclient.wait_for_result(rospy.Duration.from_sec(10.0))

	muffins= lmclient.get_result().muffin
	if len(muffins) == 0:
		rospy.logerr("no muffin found")
		exit(1)
	# pick muffin that is nearest to the frame origin (base_footprint for now)
	muffin= min(muffins, key= (lambda m: sqrt(m.primitive_poses[0].position.x**2+m.primitive_poses[0].position.y**2+m.primitive_poses[0].position.z**2)))

	rospy.loginfo("found muffin '%s'" %  muffins[0].id)
	rospy.loginfo("waiting for muffin & table to appear in PlanningScene")
	rospy.sleep(rospy.Duration.from_sec(15.0))

	rospy.loginfo("pick muffin")
	piclient.send_goal(PickServerGoal(co= muffins[0], close_gripper_partially= False))
	piclient.wait_for_result(rospy.Duration.from_sec(120.0))

	rospy.loginfo("move arm back to home pose")
	mgclient.send_goal(move_group_goal([-0.0286, 2.1635, -2.0931, -1.9421, 0.0605]) )
	mgclient.wait_for_result(rospy.Duration.from_sec(30.0))
	ret= mgclient.get_result().error_code.val
	if ret != MoveItErrorCodes.SUCCESS and ret != MoveItErrorCodes.TIMED_OUT:
		rospy.logerr("Move Arm Away failed - %s" % mgclient.get_result().error_code.val)
		exit(1)
