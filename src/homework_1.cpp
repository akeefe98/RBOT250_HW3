// Modified from "Writing-A-Tf2-Broadcaster-Cpp.html" tutorial
// https://docs.ros.org/en/foxy/Tutorials/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
// And "Writing-A-Tf2-Static-Broadcaster-Cpp.html" tutorial
// http://docs.ros.org/en/rolling/Tutorials/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    this->declare_parameter<std::string>("turtlename", "turtle");
    this->get_parameter("turtlename", turtlename_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();
    // publish the transformed message, using default buffer size of 10
    publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("homeworks/hw1/tf", 10);
    // create timer for publishing
    // setting period to 30mS to achieve at least 30Hz rate
    timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&FramePublisher::timer_callback, this));
 
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "base"; // per instructions
    t.child_frame_id = "elbow"; // per instructions

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
  size_t count_;

  geometry_msgs::msg::TransformStamped get_tf2_value()
  {
     rclcpp::Time now = this->get_clock()->now();
     geometry_msgs::msg::TransformStamped t;

     // Read message content and assign it to
     // corresponding tf variables
     t.header.stamp = now;
     t.header.frame_id = "base";  // per instructions
     t.child_frame_id = "elbow";  // per instructions

     t.transform.translation.x = 1.57;        // per instructions
     t.transform.translation.y = 3.142;       // per instructions
     t.transform.translation.z = -2 * M_PI/3; // per instructions

     tf2::Quaternion q;
     t.transform.rotation.x = 0.123;
     t.transform.rotation.y = 1.57;
     t.transform.rotation.z = 5*M_PI/6;
     t.transform.rotation.w = 1;

     // Send the transformation
     tf_broadcaster_->sendTransform(t);     
     return t;
  }

  void timer_callback()
  {
    auto message = geometry_msgs::msg::TransformStamped();
    message = get_tf2_value();
    RCLCPP_INFO(this->get_logger(), "translation xyz: '%f %f %f'", message.transform.translation.x, message.transform.translation.y, message.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "rotation xyz:    '%f %f %f'", message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z);
    publisher_->publish(message);
  }

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}