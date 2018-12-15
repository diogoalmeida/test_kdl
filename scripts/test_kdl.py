#!/usr/bin/env python
import rospy
import math
import random
import PyKDL as kdl
from kdl_parser_py import urdf


if __name__ == "__main__":
    rospy.init_node("test_kdl_py")

    if not rospy.has_param("~base_link"):
        rospy.logerr("Missing base_link")
        exit()

    if not rospy.has_param("~eef"):
        rospy.logerr("Missing eef")
        exit()

    if not rospy.has_param("~max_trials"):
        rospy.logerr("Missing max_trials")
        exit()

    base_link = rospy.get_param("~base_link")
    eef = rospy.get_param("~eef")
    max_trials = rospy.get_param("~max_trials")

    tree = urdf.treeFromParam("/robot_description")

    if not tree[0]:
        rospy.logerr("Missing /robot_description")
        exit()

    chain = tree[1].getChain(base_link, eef)
    jac_solver = kdl.ChainJntToJacSolver(chain)
    q = kdl.JntArray(chain.getNrOfJoints())
    J = kdl.Jacobian(chain.getNrOfJoints())

    rospy.loginfo("Initialized test_kdl_py")
    init_time = rospy.Time.now()
    for i in range(max_trials):
        for j in range(chain.getNrOfJoints()):
            q[j] = random.uniform(-math.pi, math.pi)

        jac_solver.JntToJac(q, J)

    rospy.loginfo("Elapsed time: " + str((rospy.Time.now() - init_time).to_sec()))
