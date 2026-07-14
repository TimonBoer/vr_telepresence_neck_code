// see arduino_parallel_classes_overview.png for a classes overview

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <boost/asio.hpp>
#include <string>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ── Latency-echo naar de Quest (UDP) ─────────────────────────────────────────
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>

// adding latency
#include <deque>
#include <chrono>

// ── Neck geometry (millimetres) ──────────────────────────────────────────────
constexpr double VERTEBRA_RADIUS = 52.0;
constexpr double VERTEBRA_HEIGHT = 7.0;
constexpr double CENTER_RADIUS = 5.0;
constexpr double ROPE_OFFSET = 20.0;
constexpr int N_VERTEBRAE = 6;
constexpr double SPINDLE_RADIUS = 20.0;
constexpr double NEUTRAL_ROPE_LENGTH = VERTEBRA_HEIGHT * 2.0 * N_VERTEBRAE;

static constexpr int TIGHTNESS = 10;
static constexpr int LEFT_NEUTRAL = 125 + TIGHTNESS;
static constexpr int RIGHT_NEUTRAL = 125 + TIGHTNESS;
static constexpr int FRONT_NEUTRAL = 125 + TIGHTNESS;
static constexpr int BACK_NEUTRAL = 125 + TIGHTNESS;

static constexpr int YAW_NEUTRAL = 60;

// ── Tightness keyframes ───────────────────────────────────────────────────────
// Each entry is {tilt_degrees, tightness_at_x_neg15, tightness_at_x_pos25}
// Must be sorted by tilt ascending.

// was not used for the latest prototype
struct TightnessKeyframe
{
    double tilt_deg, neg15, pos25;
};
static const std::vector<TightnessKeyframe> TIGHTNESS_KEYFRAMES = {
    {0.0, 0.0, 0.0},
};

std::pair<double, double> interpolateKeyframes(double tilt_rad)
{
    double tilt_deg = tilt_rad * 180.0 / M_PI;
    if (tilt_deg <= TIGHTNESS_KEYFRAMES.front().tilt_deg)
        return {TIGHTNESS_KEYFRAMES.front().neg15, TIGHTNESS_KEYFRAMES.front().pos25};
    if (tilt_deg >= TIGHTNESS_KEYFRAMES.back().tilt_deg)
        return {TIGHTNESS_KEYFRAMES.back().neg15, TIGHTNESS_KEYFRAMES.back().pos25};

    for (size_t i = 0; i + 1 < TIGHTNESS_KEYFRAMES.size(); ++i)
    {
        const auto &k0 = TIGHTNESS_KEYFRAMES[i];
        const auto &k1 = TIGHTNESS_KEYFRAMES[i + 1];
        if (tilt_deg >= k0.tilt_deg && tilt_deg <= k1.tilt_deg)
        {
            double t = (tilt_deg - k0.tilt_deg) / (k1.tilt_deg - k0.tilt_deg);
            return {k0.neg15 + t * (k1.neg15 - k0.neg15),
                    k0.pos25 + t * (k1.pos25 - k0.pos25)};
        }
    }
    return {0.0, 0.0};
}

double tightnessAtX(double x, double tightness_neg15, double tightness_pos25)
{
    double t = std::clamp((x - (-15.0)) / (25.0 - (-15.0)), 0.0, 1.0);
    return tightness_neg15 + t * (tightness_pos25 - tightness_neg15);
}

// ── Rope ──────────────────────────────────────────────────────────────────────
// Control of a single rope. Tilt, pan -> rope length -> motor angle

// First calculates the x position using the pan angle, 
// then calculates the motor offset using the tilt.

// Before the update function, tilt gets converted to rope length at x=0 and sin(tilt/12)
// So these calculations wont be repeated for all 4 ropes
class Rope
{
public:
    Rope(int motor_neutral_angle, double angular_pos, std::string name = "")
        : angular_pos_(angular_pos), motor_neutral_angle_(motor_neutral_angle),
          angle_from_curve_centre_(0.0), dist_from_curve_centre_(0.0),
          rope_length_(0.0), angle_offset_(0.0), motor_angle_(motor_neutral_angle), name_(name)
    {
        update(VERTEBRA_HEIGHT, 0.0, 0.0, 0.0, 0.0);
    }

    void update(double rope_len_x_0, double sin_th, double pan_angle, double tightness_neg15, double tightness_pos25)
    {
        // calc x position of the rope attachment point based on pan angle and angular position
        x_ = ROPE_OFFSET * std::cos(pan_angle + angular_pos_) + CENTER_RADIUS;

        // calculate the rope length based on the x position and the tilt angle
        double length_per_vertebra = rope_len_x_0 + sin_th * x_;
        rope_length_ = length_per_vertebra * 2.0 * N_VERTEBRAE;

        motor_neutral_angle_adjusted_ = motor_neutral_angle_ + tightnessAtX(x_, tightness_neg15, tightness_pos25);
        updateMotorAngle();
    }

    double getMotorAngle() const { return motor_angle_; }
    std::string getName() const { return name_; }

private:
    void updateMotorAngle()
    {
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        double length_delta = NEUTRAL_ROPE_LENGTH - rope_length_;
        angle_offset_ = (length_delta / SPINDLE_RADIUS) * RAD_TO_DEG;
        motor_angle_ = static_cast<int>(motor_neutral_angle_adjusted_ + angle_offset_);
    }

    double angular_pos_;
    int motor_neutral_angle_;
    double motor_neutral_angle_adjusted_;
    double angle_from_curve_centre_;
    double dist_from_curve_centre_;
    double rope_length_;
    double angle_offset_;
    double motor_angle_;
    double x_;
    std::string name_;
};

// ── RobotController ───────────────────────────────────────────────────────────
struct ControllerConfig
{
    double update_rate_hz = 240.0;
    double max_accel = 15.0; // rad/s², can be higher — following head motion
    double max_decel = 6.0;  // rad/s², lower — prevents overshoot snapping ropes
    double max_accel_yaw = 10000.0;
    double max_decel_yaw = 10.0;
};

struct Pose
{
    double tilt;
    double pan;
    double yaw;
};

// ── RobotController ──────────────────────────────────────────────────────────────────────
// setpoint orientation as input, ouputs a current orientation that moves to the setpoint with a limited acceleration
class RobotController
{
public:
    explicit RobotController(const ControllerConfig &cfg) : cfg_(cfg) {}

    void setSetpoint(const tf2::Vector3 &normal, double yaw)
    {
        setpoint_yaw_ = yaw;
        setpoint_normal_ = normal.normalized();
    }

    Pose update()
    {
        const double dt = 1.0 / cfg_.update_rate_hz;

        // ── Normal vector (tilt/pan) ──────────────────────────────────────────

        // Angle and axis from current normal to setpoint
        tf2::Vector3 cross = current_normal_.cross(setpoint_normal_);
        double sin_angle = cross.length();
        double cos_angle = std::clamp(current_normal_.dot(setpoint_normal_), -1.0, 1.0);
        double angle_to_target = std::atan2(sin_angle, cos_angle);

        tf2::Vector3 desired_vel{0.0, 0.0, 0.0};
        if (sin_angle > 1e-9)
        {
            tf2::Vector3 direction = cross.normalized();

            // Follow setpoint: desired velocity to close error in one step
            double desired_speed = angle_to_target / dt;

            // Clamp to maximum speed we can still brake from
            double max_stoppable_speed = std::sqrt(2.0 * cfg_.max_decel * angle_to_target);
            if (desired_speed > max_stoppable_speed)
                desired_speed = max_stoppable_speed;

            desired_vel = direction * desired_speed;
        }

        // Clamp acceleration and deceleration separately
        tf2::Vector3 dv = desired_vel - current_vel_normal_;
        double dv_mag = dv.length();
        bool decelerating = current_vel_normal_.length() > 1e-9 &&
                            current_vel_normal_.dot(dv) < 0.0;
        double max_dv = (decelerating ? cfg_.max_decel : cfg_.max_accel) * dt;
        if (dv_mag > max_dv)
            dv *= (max_dv / dv_mag);
        current_vel_normal_ += dv;

        // Integrate
        double step_angle = current_vel_normal_.length() * dt;
        if (step_angle > 1e-9)
        {
            tf2::Quaternion q(current_vel_normal_.normalized(), step_angle);
            current_normal_ = quatRotate(q, current_normal_).normalized();
        }

        // Remove the spin component parallel to the normal — it has no effect on
        // the normal direction but would re-emerge as unwanted velocity if tilt increases.
        // current_vel_normal_ -= current_normal_ * current_normal_.dot(current_vel_normal_);

        // ── Yaw ───────────────────────────────────────────────────────────────
        double yaw_error = setpoint_yaw_ - current_yaw_;

        // Follow setpoint velocity
        double desired_yaw_vel = yaw_error / dt;

        // Clamp to the maximum speed we can still brake from given remaining distance
        double max_stoppable_speed = std::sqrt(2.0 * cfg_.max_decel_yaw * std::abs(yaw_error));
        if (std::abs(desired_yaw_vel) > max_stoppable_speed)
            desired_yaw_vel = std::copysign(max_stoppable_speed, yaw_error);

        bool yaw_decelerating = (desired_yaw_vel - current_yaw_vel_) * current_yaw_vel_ < 0.0;
        double max_dyv = (yaw_decelerating ? cfg_.max_decel_yaw : cfg_.max_accel_yaw) * dt;
        double dyv = std::clamp(desired_yaw_vel - current_yaw_vel_, -max_dyv, max_dyv);
        current_yaw_vel_ += dyv;
        current_yaw_ += current_yaw_vel_ * dt;

        return getCurrentPose();
    }

    Pose getCurrentPose() const
    {
        auto [tilt, pan] = normalToTiltPan(current_normal_);
        return Pose{tilt, pan, current_yaw_};
    }

private:
    static tf2::Vector3 tiltPanToNormal(double tilt, double pan)
    {
        return tf2::Vector3(
            -std::sin(tilt) * std::sin(pan - M_PI),
            std::sin(tilt) * std::cos(pan - M_PI),
            std::cos(tilt));
    }

    std::pair<double, double> normalToTiltPan(const tf2::Vector3 &normal) const
    {
        double tilt = std::acos(std::clamp(normal.z(), -1.0, 1.0));
        double pan = std::atan2(-normal.x(), normal.y()) + M_PI;
        return {tilt, pan};
    }

    ControllerConfig cfg_;
    double current_yaw_ = 0.0;
    double current_yaw_vel_ = 0.0;
    tf2::Vector3 current_normal_ = {0.0, 0.0, 1.0};
    tf2::Vector3 current_vel_normal_ = {0.0, 0.0, 0.0};

    double setpoint_yaw_ = 0.0;
    tf2::Vector3 setpoint_normal_ = {0.0, 0.0, 1.0};

    bool setpoint_initialized_ = false;
    rclcpp::Time last_setpoint_time_;
};

// ── ROS Node ──────────────────────────────────────────────────────────────────
class ArduinoParallel_v2 : public rclcpp::Node
{
public:
    ArduinoParallel_v2() : Node("arduino_parallel_v2"), io_(), serial_(io_),
                           controller_(ControllerConfig{}),
                           ropes_({
                               Rope(LEFT_NEUTRAL, 0, "left"),
                               Rope(RIGHT_NEUTRAL, M_PI, "right"),
                               Rope(FRONT_NEUTRAL, M_PI / 2, "front"),
                               Rope(BACK_NEUTRAL, -M_PI / 2, "back"),
                           })
    {
        try
        {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial_connected_ = true;
        }
        catch (const std::exception &e)
        {
            serial_connected_ = false;
            RCLCPP_WARN(this->get_logger(), "Arduino not connected — logging only");
        }

        this->declare_parameter("latency_ms", 0.0);
        latency_ms_ = this->get_parameter("latency_ms").as_double();

        // ── Latency-echo opzetten ────────────────────────────────────────────
        // Stuurt het seq-nummer (uit frame_id) als UDP-pakketje terug naar de
        // Quest op ECHO_PORT, zodat die de round-trip-latency kan meten.
        // Quest-IP komt uit de omgevingsvariabele QUEST_IP (export QUEST_IP=...).
        echo_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        const char *quest_ip = std::getenv("QUEST_IP");
        if (echo_sock_ >= 0 && quest_ip)
        {
            std::memset(&echo_addr_, 0, sizeof(echo_addr_));
            echo_addr_.sin_family = AF_INET;
            echo_addr_.sin_port = htons(ECHO_PORT);
            if (inet_pton(AF_INET, quest_ip, &echo_addr_.sin_addr) == 1)
            {
                echo_enabled_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "Latency-echo aan: seq -> %s:%d (UDP)", quest_ip, ECHO_PORT);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "QUEST_IP ongeldig — echo uit");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "QUEST_IP niet gezet — latency-echo uit (export QUEST_IP=<ip bril>)");
        }

        quest_pub_ = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("quest/orientation", 10);

        // Subscriber just updates the setpoint.
        subscription_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "orientation", 10,
            [this](const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
            {
                // Echo het seq DIRECT terug, vóór alle servo-berekeningen, zodat
                // de meting de keten-/netwerklatency weergeeft en niet de rekentijd.

                msg->header.stamp = this->now();
                quest_pub_->publish(*msg); // echo the received orientation for logging/debugging

                RCLCPP_INFO(this->get_logger(), "Quaternion data (xyzw): (%.4f, %.4f, %.4f, %.4f)",
                            msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);

                auto [v_norm, yaw] = quaternionToNormalYaw(msg);
                setpoint_queue_.push_back({std::chrono::steady_clock::now(),
                                           v_norm,
                                           yaw,
                                           msg->header.frame_id});
            });

        // Timer drives the control loop and sends motor angles.
        const auto period = std::chrono::duration<double>(1.0 / 240.0);
        timer_ = this->create_wall_timer(period, [this]()
                                         { controlLoop(); });

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                for (const auto &p : params)
                {
                    if (p.get_name() == "latency_ms")
                    {
                        latency_ms_ = p.as_double();
                        setpoint_queue_.clear(); // flush stale data on change
                        RCLCPP_INFO(this->get_logger(),
                                    "Latency set to %.1f ms", latency_ms_);
                    }
                }
                return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
            });
    }

    ~ArduinoParallel_v2() override
    {
        if (echo_sock_ >= 0)
            close(echo_sock_);
    }

private:
    static constexpr int ECHO_PORT = 5006; // moet matchen met HMD_ECHO_PORT in de Quest-app

    // ── Echo-state ───────────────────────────────────────────────────────────
    int echo_sock_ = -1;
    bool echo_enabled_ = false;
    sockaddr_in echo_addr_{};

    // Haalt het seq-nummer uit frame_id ("quest_imu:<seq>") en stuurt het als
    // 32-bit network-order integer terug naar de Quest. Geen seq -> niets doen.
    void echo_seq(const std::string &frame_id)
    {
        if (!echo_enabled_)
            return;
        auto pos = frame_id.find(':');
        if (pos == std::string::npos)
            return;
        try
        {
            uint32_t seq = static_cast<uint32_t>(std::stoul(frame_id.substr(pos + 1)));
            uint32_t seq_net = htonl(seq);
            sendto(echo_sock_, &seq_net, sizeof(seq_net), 0,
                   reinterpret_cast<sockaddr *>(&echo_addr_), sizeof(echo_addr_));
        }
        catch (...)
        {
            // geen geldig seq in frame_id -> overslaan
        }
    }

    void controlLoop()
    {
        auto now = std::chrono::steady_clock::now();
        auto delay = std::chrono::duration<double, std::milli>(latency_ms_);

        while (!setpoint_queue_.empty() &&
               (now - setpoint_queue_.front().received_at) >= delay)
        {
            auto &sp = setpoint_queue_.front();
            echo_seq(sp.frame_id); // ← add this
            controller_.setSetpoint(sp.normal, sp.yaw);
            setpoint_queue_.pop_front();
        }

        Pose pose = controller_.update();

        RCLCPP_INFO(this->get_logger(), "tilt: %.4f  pan: %.4f  yaw: %.4f",
                    pose.tilt, pose.pan, pose.yaw);

        auto [tightness_neg15, tightness_pos25] = interpolateKeyframes(pose.tilt);

        double th = pose.tilt/12;
        double rope_len_x_0 = VERTEBRA_RADIUS - 45 * cos(th);
        double sin_th = sin(th);

        for (auto &rope : ropes_)
            rope.update(rope_len_x_0, sin_th, pose.pan, tightness_neg15, tightness_pos25);

        sendMotorAngles(pose.yaw, ropes_);
    }

    void sendMotorAngles(double yaw, const std::vector<Rope> &ropes)
    {
        std::vector<uint8_t> motor_angles;
        motor_angles.reserve(ropes.size() + 2);

        for (const auto &rope : ropes)
        {
            uint8_t angle = static_cast<uint8_t>(std::clamp(rope.getMotorAngle(), 0.0, 180.0));
            motor_angles.push_back(angle);
            RCLCPP_INFO(this->get_logger(), "%s: %d", rope.getName().c_str(), angle);
        }

        auto yaw_angle = static_cast<uint8_t>(std::clamp(yaw * 180.0 / M_PI + YAW_NEUTRAL, 0.0, 180.0));
        motor_angles.push_back(yaw_angle);
        RCLCPP_INFO(this->get_logger(), "Yaw: %d", yaw_angle);

        motor_angles.push_back(181);
        if (serial_connected_)
            boost::asio::write(serial_, boost::asio::buffer(motor_angles));
    }

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

    std::tuple<tf2::Vector3, double> quaternionToNormalYaw(const geometry_msgs::msg::QuaternionStamped::SharedPtr q_msg)
    {
        tf2::Quaternion q;
        tf2::fromMsg(q_msg->quaternion, q);

        // get normal vector by applying rotation to vector pointing up (0, 0, 1)
        tf2::Vector3 v_norm = quatRotate(q, tf2::Vector3(0, 0, 1));

        double tilt = std::clamp(acos(std::clamp(v_norm.z(), -1.0, 1.0)), 0.0, M_PI / 2);
        double pan = -atan2(v_norm.x(), v_norm.y()) + M_PI;

        tf2::Quaternion tilt_q;
        tf2::Quaternion pan_q;

        tilt_q.setRPY(tilt, 0, 0);
        pan_q.setRPY(0, 0, pan);

        v_norm = quatRotate(pan_q*tilt_q, tf2::Vector3(0, 0, 1));

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

        return {v_norm, yaw};
    }

    rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quest_pub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    RobotController controller_;
    std::vector<Rope> ropes_;
    bool serial_connected_ = false;

    struct TimestampedSetpoint
    {
        std::chrono::steady_clock::time_point received_at;
        tf2::Vector3 normal;
        double yaw;
        std::string frame_id; // ← add this
    };

    double latency_ms_ = 0.0;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    std::deque<TimestampedSetpoint> setpoint_queue_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoParallel_v2>());
    rclcpp::shutdown();
    return 0;
}