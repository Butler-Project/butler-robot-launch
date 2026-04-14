#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class FootprintPublisher : public rclcpp::Node
{
public:
  explicit FootprintPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("footprint_publisher", options)
  {
    // Parameters
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter<std::string>("footprint_frame", "base_footprint");
    this->declare_parameter<double>("publish_rate", 30.0);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    footprint_frame_ = this->get_parameter("footprint_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&FootprintPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Footprint Publisher (C++) started: %s -> %s", 
                odom_frame_.c_str(), footprint_frame_.c_str());
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        odom_frame_, base_link_frame_,
        tf2::TimePointZero);

      // Extract yaw
      tf2::Quaternion q;
      tf2::fromMsg(transform_stamped.transform.rotation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // Create new transform
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = odom_frame_;
      t.child_frame_id = footprint_frame_;

      t.transform.translation.x = transform_stamped.transform.translation.x;
      t.transform.translation.y = transform_stamped.transform.translation.y;
      t.transform.translation.z = 0.0; // Flat on ground

      tf2::Quaternion q_new;
      q_new.setRPY(0, 0, yaw); // Pure yaw rotation
      t.transform.rotation = tf2::toMsg(q_new);

      // Send transform
      tf_broadcaster_->sendTransform(t);

    } catch (const tf2::TransformException & ex) {
      // Log only occasionally to avoid spam
      RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                   odom_frame_.c_str(), base_link_frame_.c_str(), ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string odom_frame_;
  std::string base_link_frame_;
  std::string footprint_frame_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(FootprintPublisher)

#ifndef COMPONENT_ONLY
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootprintPublisher>());
  rclcpp::shutdown();
  return 0;
}
#endif
