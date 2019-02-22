#ifndef DD_GAZEBO_PLUGINS_ROUTER_H
#define DD_GAZEBO_PLUGINS_ROUTER_H

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo_plugins/PubQueue.h>

#include <sdf/Param.hh>

#include <geometry_msgs/PoseStamped.h>

#include <dd_gazebo_plugins/Router.h>

namespace gazebo
{
class Router : public ModelPlugin
{
private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;

  common::Time last_time_;

  boost::thread callback_queue_thread_;

  PubQueue<dd_gazebo_plugins::Router>::Ptr pub_queue_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  // Rate control
  double update_rate_;

  ros::CallbackQueue router_queue_;

  boost::thread deferred_load_thread_;

  /// \brief A mutex to lock access to fields
  /// that are used in message callbacks
  boost::mutex lock_;

  std::string robot_namespace_;
  std::string topic_name_;

  ros::NodeHandle* nh_;

  ros::Publisher pub_;
  // ros publish multi queue, prevents publish() blocking
  PubMultiQueue pmq_;

  geometry_msgs::PoseStamped pose_;
  double range_;

  dd_gazebo_plugins::Router msg_;

public:
  Router();

  virtual ~Router();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();

private:
  void LoadThread();

  void RouterQueueThread();
};
}  // namespace gazebo
#endif  // DD_GAZEBO_PLUGINS_ROUTER_H