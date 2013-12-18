#ifndef _ROBOT_JOINT_POSITION_CONTROL_PLUGIN_H_
#define _ROBOT_JOINT_POSITION_CONTROL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

// custom includes


// forward declarations


/*
 * Add to robot's SDF model:
 *
 * <plugin name="RobotJointPositionControlPlugin" filename="libRobotJointPositionControlPlugin.so">
 *   <robotNamespace>/robots/mywonderfulrobot</robotNamespace>
 *   <!-- Write/Read as seen from the user of the robot -->
 *   <jointsReadTopic>get_joint_positions</jointsReadTopic>
 *   <jointsWriteTopic>set_joint_positions</jointsWriteTopic>
 * </plugin>
 *
 */

namespace gazebo {
  class RobotJointPositionControlPlugin
    : public ModelPlugin
  {
    public:
      // enums
  
      // typedefs
  
      // const static member variables
   
      // static utility functions
  
  
      // constructors
  
      // overwritten methods
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  
      // methods
      void OnUpdate();
  
      // variables
  
  
    private:
      // methods
      bool loadParams(sdf::ElementPtr _sdf);
      //void cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& pose);
      void jointsWriteCallback(const sensor_msgs::JointState::ConstPtr& joints);
      void singleJointsWriteCallback(const sensor_msgs::JointState::ConstPtr& joints, int jointIndex);
      void updateRobotState();
      void publishRobotState();
  
      // variables
      // gazebo
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;
      physics::Joint_V m_joints;

      // ros
      ros::NodeHandle* m_node;
      /*
      ros::Subscriber m_cartesianWriteTopicSub;
      ros::Publisher m_cartesianReadTopicPub;
      */
      ros::Subscriber m_jointsWriteTopicSub;
      ros::Publisher m_jointsReadTopicPub;
      std::string m_nodeName;
      std::string m_robotNamespaceName;
      /*
      std::string m_cartesianReadTopicName;
      std::string m_cartesianWriteTopicName;
      */
      std::string m_jointsReadTopicName;
      std::string m_jointsWriteTopicName;
      //geometry_msgs::Pose m_cartesianPoseCurrent;
      sensor_msgs::JointState m_jointsCurrent;
  
      std::vector<ros::Subscriber> m_singleJointWriteTopicSub;
      std::vector<ros::Publisher> m_singleJointReadTopicSub;
      std::vector<sensor_msgs::JointState> m_singleJointCurrent;
  
  };
}

#endif // _ROBOT_JOINT_POSITION_CONTROL_PLUGIN_H_
