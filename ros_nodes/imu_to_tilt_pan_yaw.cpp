// calculates and logs the tilt pan and yaw angles of the quaternion published on the imu/orientation topic

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class ImuToTiltPanYaw : public rclcpp::Node
{
public:
    ImuToTiltPanYaw() : Node("imu_to_tilt_pan_yaw")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "imu/orientation", 10,
            [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
            {   
                auto buff = msg->quaternion.x;
                msg->quaternion.x = msg->quaternion.z;
                msg->quaternion.z = buff;
                auto [tilt, pan, yaw] = quaternionToTiltPanYaw(msg);
                double RAD_TO_DEG = 180.0 / M_PI;
                RCLCPP_INFO(this->get_logger(), "Tilt: %.2f deg, Pan: %.2f deg, Yaw: %.2f deg",
                            tilt * RAD_TO_DEG, pan * RAD_TO_DEG, yaw * RAD_TO_DEG);
            });
    }


private:
std::tuple<double, double, double> quaternionToTiltPanYaw(const geometry_msgs::msg::QuaternionStamped::SharedPtr q_msg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(q_msg->quaternion, q);

        // get normal vector by applying rotation to vector pointing up (0, 0, 1)
        tf2::Vector3 v_norm = quatRotate(q, tf2::Vector3(0, 0, 1));

        double tilt = std::clamp(acos(std::clamp(v_norm.z(), -1.0, 1.0)), 0.0, M_PI / 2);
        double pan = -atan2(v_norm.x(), v_norm.y()) + M_PI;

        // get forward vector by applying rotation to the vector pointing forward (1, 0, 0)
        tf2::Vector3 v_forward = quatRotate(q, tf2::Vector3(1, 0, 0));

        // get quaternion of only the tilt and pan rotation, having 0 yaw rotation
        tf2::Vector3 up(0, 0, 1);
        tf2::Quaternion tilt_pan_rot = tf2::shortestArcQuatNormalize2(up, v_norm);

        // rotating vector (1, 0, 0) results in the vector with 0 yaw
        tf2::Vector3 expected_forward = quatRotate(tilt_pan_rot, tf2::Vector3(1, 0, 0));

        // calculate angle between expected forward and actual forward vector.
        double yaw = std::acos(std::clamp(v_forward.dot(expected_forward), -1.0, 1.0));

        // determine the sign of the yaw angle by checking the direction of the cross product
        tf2::Vector3 cross = expected_forward.cross(v_forward);
        if (cross.dot(v_norm) < 0)
            yaw = -yaw;

        return {tilt, pan, yaw};
    }

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToTiltPanYaw>());
  rclcpp::shutdown();
  return 0;
}