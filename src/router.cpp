#include <dd_gazebo_plugins/router.h>

#include <regex>
#include <limits>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Router)

Router::Router()
{
  msg_.id = rand() % std::numeric_limits<int>::max();
}

Router::~Router()
{
  this->update_connection_.reset();
  // Finalize the controller
  nh_->shutdown();
  callback_queue_thread_.join();
  delete nh_;
}

// Load the controller
void Router::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  model_ = _parent;
  world_ = _parent->GetWorld();
  sdf_ = _sdf;

  // ros callback queue for processing subscription
  deferred_load_thread_ = boost::thread(boost::bind(&Router::LoadThread, this));
}

// Load the controller
void Router::LoadThread()
{
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load "
                     "plugin. Load the Gazebo system plugin "
                     "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load parameters
  robot_namespace_ = "";
  if (sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->Get<std::string>("robotNamespace") + "/";
  }

  if (!sdf_->HasElement("topicName"))
  {
    ROS_INFO_NAMED("router", "router plugin missing <topicName>, defaults to "
                             "/default_router");
    topic_name_ = "/default_router";
  }
  else
  {
    topic_name_ = sdf_->Get<std::string>("topicName");
  }

  if (!sdf_->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("router", "router plugin missing <updateRate>, defaults to 0.0"
                              " (as fast as possible)");
    update_rate_ = 0.0;
  }
  else
  {
    update_rate_ = sdf_->GetElement("updateRate")->Get<double>();
  }

  if (sdf_->HasElement("range"))
  {
    range_ = sdf_->Get<double>("range");
  }
  else
  {
    ROS_INFO_NAMED("router", "router plugin missing <range>, defaults to 20");
    range_ = 20;
  }

  nh_ = new ros::NodeHandle(robot_namespace_);

  // publish multi queue
  pmq_.startServiceThread();

  // if topic name specified as empty, do not publish
  if (topic_name_ != "")
  {
    // ros::Duration(((double)(rand() % 1000)) / 1000.0).sleep();

    // ros::spinOnce();

    // ros::master::V_TopicInfo topics;
    // if (!ros::master::getTopics(topics))
    // {
    //   ROS_ERROR_NAMED("router", "Could not get topics");
    //   exit(1);
    // }

    // if (topic_name_[0] != '/')
    // {
    //   topic_name_ = "/" + topic_name_;
    // }

    // std::regex topic_name_regex(topic_name_ + "_[0-9]+$");

    // std::vector<int> used_numbers;

    // for (ros::master::TopicInfo topic : topics)
    // {
    //   // if (topic.getNumPublishers() == 0)
    //   // {
    //   //   continue;
    //   // }

    //   std::string topic_name = topic.name;

    //   if (std::regex_match(topic_name, topic_name_regex))
    //   {
    //     used_numbers.push_back(std::stoi(topic_name.substr(topic_name_.size() + 1)));
    //   }
    // }

    // std::sort(used_numbers.begin(), used_numbers.end());

    // std::string end_num = "_0";
    // if (!used_numbers.empty())
    // {
    //   end_num = "_" + std::to_string(used_numbers[used_numbers.size() - 1] + 1);
    // }

    pub_queue_ = pmq_.addPub<dd_gazebo_plugins::Router>();
    pub_ = nh_->advertise<dd_gazebo_plugins::Router>(topic_name_, 1);
  }

  // Initialize the controller
#if GAZEBO_MAJOR_VERSION >= 8
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif

  callback_queue_thread_ = boost::thread(boost::bind(&Router::RouterQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&Router::UpdateChild, this));
}

void Router::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = world_->SimTime();
#else
  common::Time cur_time = world_->GetSimTime();
#endif

  // rate control
  if (update_rate_ > 0 && (cur_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }

  if (pub_.getNumSubscribers() > 0 && topic_name_ != "")
  {
    ignition::math::Pose3d pose;
    // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
    pose = model_->WorldPose();
#else
    pose = model_->GetWorldPose().Ign();
#endif

    msg_.header.stamp.sec = cur_time.sec;
    msg_.header.stamp.nsec = cur_time.nsec;

    msg_.pose.position.x = pose.Pos().X();
    msg_.pose.position.y = pose.Pos().Y();
    msg_.pose.position.z = pose.Pos().Z();
    msg_.pose.orientation.x = pose.Rot().X();
    msg_.pose.orientation.y = pose.Rot().Y();
    msg_.pose.orientation.z = pose.Rot().Z();
    msg_.pose.orientation.w = pose.Rot().W();

    msg_.range = range_;

    {
      boost::mutex::scoped_lock lock(lock_);
      // publish to ros
      pub_queue_->push(msg_, pub_);
    }

    // save last time stamp
    last_time_ = cur_time;
  }
}

void Router::RouterQueueThread()
{
  static const double timeout = 0.01;

  while (nh_->ok())
  {
    router_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo