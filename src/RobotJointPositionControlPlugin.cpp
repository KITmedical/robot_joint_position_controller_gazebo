#include "RobotJointPositionControlPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

// custom includes


namespace gazebo
{
/*---------------------------------- public: -----------------------------{{{-*/
  void
  RobotJointPositionControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
  {
    std::cout << "------------------- RobotJointPositionControlPlugin -------------------" << std::endl;

    m_model = _parent;

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("RobotJointPositionControlPlugin: A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    if (!loadParams(_sdf)) {
      ROS_FATAL_STREAM("Error during loadParams");
      return;
    }


    m_node = new ros::NodeHandle(m_nodeName);
    /*
    m_cartesianWriteTopicSub = m_node->subscribe<geometry_msgs::Pose>(m_cartesianWriteTopicName, 1, &RobotJointPositionControlPlugin::cartesianWriteCallback, this);
    m_cartesianReadTopicPub = m_node->advertise<geometry_msgs::Pose>(m_cartesianReadTopicName, 1);
    */
    m_jointsWriteTopicSub = m_node->subscribe<sensor_msgs::JointState>(m_jointsWriteTopicName, 1, &RobotJointPositionControlPlugin::jointsWriteCallback, this);
    m_jointsReadTopicPub = m_node->advertise<sensor_msgs::JointState>(m_jointsReadTopicName, 1);

    std::string pluginName = _sdf->GetAttribute("name")->GetAsString();

    physics::Joint_V joints = m_model->GetJoints();
    std::cout << pluginName << " joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < joints.size(); jointIdx++) {
      physics::JointPtr currJoint = joints[jointIdx];
      std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
      m_joints.push_back(currJoint);
    }

    m_jointsCurrent.position.resize(m_joints.size(), 0);
    m_jointsCurrent.velocity.resize(m_joints.size(), 0);
    m_jointsCurrent.effort.resize(m_joints.size(), 0);

    // Listen to the update event. This event is broadcast every simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotJointPositionControlPlugin::OnUpdate, this));
  }

  void
  RobotJointPositionControlPlugin::OnUpdate()
  {
    updateRobotState();
    publishRobotState();
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
  bool
  RobotJointPositionControlPlugin::loadParams(sdf::ElementPtr _sdf)
  {
    m_nodeName = _sdf->GetParent()->Get<std::string>("name");

    m_robotNamespaceName = _sdf->GetElement("robotNamespace")->Get<std::string>();
    /*
    m_cartesianReadTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("cartesianReadTopic")->Get<std::string>();
    m_cartesianWriteTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("cartesianWriteTopic")->Get<std::string>();
    */
    m_jointsReadTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("jointsReadTopic")->Get<std::string>();
    m_jointsWriteTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("jointsWriteTopic")->Get<std::string>();

    return true;
  }

  /*
  void
  RobotJointPositionControlPlugin::cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
  {
    //std::cout << "cartesianWriteCallback: poseMsg=" << *poseMsg << std::endl;
    tf::Pose tfpose;
    tf::poseMsgToTF(*poseMsg, tfpose);
    tf::Vector3 tforigin = tfpose.getOrigin();
    tf::Quaternion tforientation = tfpose.getRotation();

    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, joints.j[jointIdx]);
    }
  }
  */

  void
  RobotJointPositionControlPlugin::jointsWriteCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
  {
    //std::cout << "jointsWriteCallback: jointsMsg=" << *jointsMsg << std::endl;
    if (jointsMsg->position.size() != m_jointsCurrent.position.size()) {
      ROS_WARN("Wrong number of joints received. Ignoring message.");
      return;
    }
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, jointsMsg->position[jointIdx]);
    }
  }

  void
  RobotJointPositionControlPlugin::updateRobotState()
  {
    // joints
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      m_jointsCurrent.position[jointIdx] = currJoint->GetAngle(0).Radian();
      m_jointsCurrent.velocity[jointIdx] = currJoint->GetVelocity(0);
    }

    // cartesian
  }

  void
  RobotJointPositionControlPlugin::publishRobotState()
  {
    //m_cartesianReadTopicPub.publish(m_cartesianPoseCurrent);
    m_jointsReadTopicPub.publish(m_jointsCurrent);
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotJointPositionControlPlugin)
}
