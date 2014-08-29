#include "RobotJointPositionControlPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

// custom includes
#include <ahbstring.h>

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

    m_DOFs = 0;
    physics::Joint_V joints = m_model->GetJoints();
    std::cout << pluginName << " joints (size=" << joints.size() << "):" << std::endl;
    for (size_t modelJointIdx = 0; modelJointIdx < joints.size(); modelJointIdx++) {
      physics::JointPtr currJoint = joints[modelJointIdx];

      if (!m_modelPrefix.empty()) {
        std::vector<std::string> prefixes;
        boost::split(prefixes, m_modelPrefix, boost::is_any_of("\n\t "));
        bool matches_any_prefix = false;
        for (std::vector<std::string>::const_iterator prefixIter = prefixes.begin(); prefixIter != prefixes.end(); ++prefixIter) {
          if (ahb::string::startswith(currJoint->GetName(), *prefixIter)) {
            matches_any_prefix = true;
            break;
          }
        }

        if (!matches_any_prefix) {
          std::cout << modelJointIdx << " name=" << currJoint->GetName() << " not part of model (modelPrefix=" << m_modelPrefix << ")" << std::endl;
          continue;
        }
      }

      bool fixedJoint = true;
      for (unsigned jointDOFIdx = 0; jointDOFIdx < currJoint->GetAngleCount(); jointDOFIdx++) {
        if (currJoint->GetLowerLimit(jointDOFIdx) != currJoint->GetUpperLimit(jointDOFIdx)) {
          fixedJoint = false;
          break;
        }
      }

      if (fixedJoint) {
        std::cout << modelJointIdx << " name=" << currJoint->GetName() << " is fixed joint" << std::endl;
        continue;
      }

      size_t jointIdx = m_moveableJoints.size();
      m_DOFs += currJoint->GetAngleCount();
      m_moveableJoints.push_back(currJoint);
      std::string jointTopicName = currJoint->GetName();
      ahb::string::replace(jointTopicName, "::", "_");

      std::cout << modelJointIdx << " name=" << currJoint->GetName()
                << " DOF=" << currJoint->GetAngleCount()
                << " limits=(";
      for (size_t jointDOFIdx = 0; jointDOFIdx < currJoint->GetAngleCount(); jointDOFIdx++) {
        std::cout << currJoint->GetLowerLimit(jointDOFIdx).Radian() << " ";
      }
      std::cout << ") - (";
      for (size_t jointDOFIdx = 0; jointDOFIdx < currJoint->GetAngleCount(); jointDOFIdx++) {
        std::cout << currJoint->GetUpperLimit(jointDOFIdx).Radian() << " ";
      }
      std::cout << ") jointIndex=" << jointIdx
                << " jointTopicName=" << jointTopicName
                << std::endl;

      m_singleJointWriteTopicSub.push_back(m_node->subscribe<sensor_msgs::JointState>(m_robotNamespaceName + "/set_" + jointTopicName, 1, boost::bind(&RobotJointPositionControlPlugin::singleJointsWriteCallback, this, _1, jointIdx)));
      m_singleJointReadTopicSub.push_back(m_node->advertise<sensor_msgs::JointState>(m_robotNamespaceName + "/get_" + jointTopicName, 1));
      m_singleJointCurrent.push_back(sensor_msgs::JointState());
      m_singleJointCurrent[jointIdx].position.resize(currJoint->GetAngleCount(), 0);
      m_singleJointCurrent[jointIdx].velocity.resize(currJoint->GetAngleCount(), 0);
      m_singleJointCurrent[jointIdx].effort.resize(currJoint->GetAngleCount(), 0);
    }

    m_jointsCurrent.name.resize(m_DOFs, "");
    m_jointsCurrent.position.resize(m_DOFs, 0);
    m_jointsCurrent.velocity.resize(m_DOFs, 0);
    m_jointsCurrent.effort.resize(m_DOFs, 0);

    unsigned jointIdx = 0;
    unsigned jointDOFIdx = 0;
    for (size_t dofIdx = 0; dofIdx < m_DOFs; dofIdx++) {
      if (jointDOFIdx == m_moveableJoints[jointIdx]->GetAngleCount()) {
        jointIdx++;
        jointDOFIdx = 0;
      }
      physics::JointPtr currJoint = m_moveableJoints[jointIdx];
      m_jointsCurrent.name[dofIdx] = currJoint->GetName() + ahb::string::toString(jointDOFIdx);
    }

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
    if (_sdf->HasElement("modelPrefixOnly")) {
      m_modelPrefix = _sdf->GetElement("modelPrefixOnly")->Get<std::string>();
    }

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

    for (size_t dofIdx = 0; dofIdx < m_DOFs; dofIdx++) {
      m_moveableJoints[dofIdx]->SetPosition(0, joints.j[dofIdx]);
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

    {
      unsigned jointIdx = 0;
      unsigned jointDOFIdx = 0;
      for (size_t dofIdx = 0; dofIdx < m_DOFs; dofIdx++) {
        if (jointDOFIdx == m_moveableJoints[jointIdx]->GetAngleCount()) {
          jointIdx++;
          jointDOFIdx = 0;
        }
        //printf("%d %d: %lf -> %lf %lf\n", jointIdx, jointDOFIdx, jointsMsg->position[dofIdx], m_moveableJoints[jointIdx]->GetLowerLimit(jointDOFIdx).Radian(), m_moveableJoints[jointIdx]->GetUpperLimit(jointDOFIdx).Radian());
        if (jointsMsg->position[dofIdx] < m_moveableJoints[jointIdx]->GetLowerLimit(jointDOFIdx).Radian()) {
          ROS_FATAL_STREAM("Joint" << jointIdx << " below joint limit (" << m_moveableJoints[jointIdx]->GetLowerLimit(jointDOFIdx).Radian() << "): " << jointsMsg->position[dofIdx] << ". Will not move robot at all.\n");
          return;
        } else if (jointsMsg->position[dofIdx] > m_moveableJoints[jointIdx]->GetUpperLimit(jointDOFIdx).Radian()) {
          ROS_FATAL_STREAM("Joint" << jointIdx << " above joint limit (" << m_moveableJoints[jointIdx]->GetUpperLimit(jointDOFIdx).Radian() << "): " << jointsMsg->position[dofIdx] << ". Will not move robot at all.\n");
          return;
        }
        jointDOFIdx++;
      }
    }

    {
      unsigned jointIdx = 0;
      unsigned jointDOFIdx = 0;
      for (size_t dofIdx = 0; dofIdx < m_DOFs; dofIdx++) {
        if (jointDOFIdx == m_moveableJoints[jointIdx]->GetAngleCount()) {
          jointIdx++;
          jointDOFIdx = 0;
        }
        m_moveableJoints[jointIdx]->SetPosition(jointDOFIdx, jointsMsg->position[dofIdx]);
        jointDOFIdx++;
      }
    }
  }

  void
  RobotJointPositionControlPlugin::singleJointsWriteCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg, int jointIndex)
  {
    //std::cout << "singleJointsWriteCallback: jointIndex=" << jointIndex << " jointsMsg=" << *jointsMsg << std::endl;
 
    if (jointsMsg->position.size() != m_moveableJoints[jointIndex]->GetAngleCount()) {
      ROS_WARN("Wrong number of joints received. Ignoring message.");
      return;
    }

    for (size_t jointDOFIdx = 0; jointDOFIdx < m_moveableJoints[jointIndex]->GetAngleCount(); jointDOFIdx++) {
      if (jointsMsg->position[jointDOFIdx] < m_moveableJoints[jointIndex]->GetLowerLimit(jointDOFIdx).Radian()) {
        ROS_FATAL_STREAM("Joint" << jointIndex << " below joint limit (" << m_moveableJoints[jointIndex]->GetLowerLimit(jointDOFIdx).Radian() << "): " << jointsMsg->position[jointDOFIdx] << ". Will not move robot at all.\n");
        return;
      } else if (jointsMsg->position[jointDOFIdx] > m_moveableJoints[jointIndex]->GetUpperLimit(jointDOFIdx).Radian()) {
        ROS_FATAL_STREAM("Joint" << jointIndex << " above joint limit (" << m_moveableJoints[jointIndex]->GetUpperLimit(jointDOFIdx).Radian() << "): " << jointsMsg->position[jointDOFIdx] << ". Will not move robot at all.\n");
        return;
      }
    }

    for (size_t jointDOFIdx = 0; jointDOFIdx < m_moveableJoints[jointIndex]->GetAngleCount(); jointDOFIdx++) {
      m_moveableJoints[jointIndex]->SetPosition(jointDOFIdx, jointsMsg->position[jointDOFIdx]);
    }
  }

  void
  RobotJointPositionControlPlugin::updateRobotState()
  {
    // joints
    unsigned jointIdx = 0;
    unsigned jointDOFIdx = 0;
    for (size_t dofIdx = 0; dofIdx < m_DOFs; dofIdx++) {
      if (jointDOFIdx == m_moveableJoints[jointIdx]->GetAngleCount()) {
        jointIdx++;
        jointDOFIdx = 0;
      }
      //std::cout << jointIdx << ":" << jointDOFIdx << std::endl;
      physics::JointPtr currJoint = m_moveableJoints[jointIdx];
      m_jointsCurrent.position[dofIdx] = currJoint->GetAngle(jointDOFIdx).Radian();
      m_jointsCurrent.velocity[dofIdx] = currJoint->GetVelocity(jointDOFIdx);
      m_singleJointCurrent[jointIdx].position[jointDOFIdx] = currJoint->GetAngle(jointDOFIdx).Radian();
      m_singleJointCurrent[jointIdx].velocity[jointDOFIdx] = currJoint->GetVelocity(jointDOFIdx);
      jointDOFIdx++;
    }

    // cartesian
  }

  void
  RobotJointPositionControlPlugin::publishRobotState()
  {
    //m_cartesianReadTopicPub.publish(m_cartesianPoseCurrent);
    m_jointsReadTopicPub.publish(m_jointsCurrent);

    for (size_t jointIdx = 0; jointIdx < m_singleJointCurrent.size(); jointIdx++) {
      m_singleJointReadTopicSub[jointIdx].publish(m_singleJointCurrent[jointIdx]);
    }
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotJointPositionControlPlugin)
}
