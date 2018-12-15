#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <random>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_kdl_cpp");
  ros::NodeHandle nh("~");

  std::string base_link, eef;
  if (!nh.getParam("base_link", base_link))
  {
    ROS_ERROR("Missing base_link");
    return -1;
  }

  if (!nh.getParam("eef", eef))
  {
    ROS_ERROR("Missing eef");
    return -1;
  }

  int max_trials;
  if (!nh.getParam("max_trials", max_trials))
  {
    ROS_ERROR("Missing max_trials");
    return -1;
  }

  if (max_trials <= 0)
  {
    ROS_ERROR("max_trials <= 0");
    return -1;
  }

  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Missing /robot_description");
    return -1;
  }
  KDL::Tree tree;
  KDL::Chain chain;
  kdl_parser::treeFromUrdfModel(model, tree);
  tree.getChain(base_link, eef, chain);
  KDL::ChainJntToJacSolver jac_solver(chain);
  KDL::JntArray q(chain.getNrOfJoints());
  std::uniform_real_distribution<double> unif(-M_PI, M_PI);
  std::default_random_engine re;
  KDL::Jacobian out(chain.getNrOfJoints());
  ROS_INFO("Initialized test_kdl_cpp!");
  ros::Time init = ros::Time::now();

  for (unsigned int t = 0; t < max_trials; t++)
  {
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      q(i) = unif(re);
    }

    jac_solver.JntToJac(q, out);
  }

  ROS_INFO_STREAM("Elapsed time: " << (ros::Time::now() - init).toSec());

  return 0;
}
