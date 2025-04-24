#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>
#include <chrono>

//------------------------------------------------------------------------------
//                              CYCLIC OBSTACLE
//------------------------------------------------------------------------------
class CyclicObstacle
{
public:
    CyclicObstacle(int center, double sigma, int limit) noexcept;
    double operator[](int idx) const;

private:
    int center_;
    double sigma_;
    int limit_;
};

CyclicObstacle::CyclicObstacle(int center, double sigma, int limit) noexcept:
center_(center), sigma_(sigma), limit_(limit)
{
}

double CyclicObstacle::operator[](int idx)const
{
 
    // Wraparound
    if(idx > center_ + limit_/2)
    {
        idx = idx - limit_;
    }
    else if(idx < center_ - limit_/2)
    {
        idx = limit_  + idx + 1;
    }

    // Makes index relative to center
    idx = idx - center_;


    return 1.0 - exp(0.0 - pow(idx-center_, 2) / (2*pow(sigma_, 2)));
}


//------------------------------------------------------------------------------
//                              LIDAR CONFIG
//------------------------------------------------------------------------------
struct FakeLidarConfig
{
public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;
    double sigma;

    void declare(rclcpp::Node *node);
    void update(rclcpp::Node *node);
    void print(rclcpp::Node *node);

    double get_scan_step() const;
    int get_scan_period_ms() const;
    double get_scaled_sample(double scale) const;

private:
    constexpr static const char* PARAM_MIN_RANGE = "min_range";
    constexpr static const char* PARAM_MAX_RANGE = "max_range";
    constexpr static const char* PARAM_MIN_ANGLE = "min_angle";
    constexpr static const char* PARAM_MAX_ANGLE = "max_angle";
    constexpr static const char* PARAM_SAMPLE_COUNT = "sample_count";
    constexpr static const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";
    constexpr static const char* PARAM_SIGMA = "sigma";

    constexpr static const double DEFAULT_MIN_RANGE = 0.2;
    constexpr static const double DEFAULT_MAX_RANGE = 2.0;
    constexpr static const double DEFAULT_MIN_ANGLE = -M_PI;
    constexpr static const double DEFAULT_MAX_ANGLE = M_PI;
    constexpr static const int DEFAULT_SAMPLE_COUNT = 360;
    constexpr static const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    constexpr static const double DEFAULT_SIGMA = 1.0;

    constexpr static const char* DESCRIPTION_MIN_RANGE =
    "minimum range value [m]";
    constexpr static const char* DESCRIPTION_MAX_RANGE =
    "maximum range value [m]";
    constexpr static const char* DESCRIPTION_MIN_ANGLE =
    "start angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_MAX_ANGLE =
    "end angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_SAMPLE_COUNT =
    "Number of samples per full laser scan";
    constexpr static const char* DESCRIPTION_SAMPLING_FREQUENCY =
    "Number of full Scans per second.";
    constexpr static const char* DESCRIPTION_SIGMA =
    "Standard deviation of obstacle (it is essentialy normal distribution pushed into circle).";

    constexpr static const double epsilon = 0.00001;
};

void FakeLidarConfig::declare(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
    descriptor.description = DESCRIPTION_SIGMA;
    node->declare_parameter(PARAM_SIGMA, DEFAULT_SIGMA, descriptor);
}

void FakeLidarConfig::update(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
    this->sigma = node->get_parameter(PARAM_SIGMA).as_double();
}

void FakeLidarConfig::print(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY, this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SIGMA, this->sigma);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());
}

double FakeLidarConfig::get_scaled_sample(double scale) const
{
    // Range is not inclusive we need substract small number (epsilon)
    // so that scale 1.0 will return valid sample.
    return range.first + (range.second - range.first - epsilon) * scale;
}

int FakeLidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double FakeLidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

//------------------------------------------------------------------------------
//                             FAKE LIDAR NODE
//------------------------------------------------------------------------------
class FakeLidarNode: public rclcpp::Node
{
public:
    FakeLidarNode();

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> frame_brodcaster_;
    rclcpp::TimerBase::SharedPtr main_clock_;
    sensor_msgs::msg::LaserScan scan_;
    FakeLidarConfig cfg_;
    int center_ = 0;

    void publish_scan_();
    void prepare_msg_config_();


};

FakeLidarNode::FakeLidarNode(): rclcpp::Node("fake_lidar"){
    cfg_.declare(this);
    cfg_.update(this);
    cfg_.print(this);
    prepare_msg_config_();
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan", 10);
    main_clock_ = this->create_wall_timer(
        std::chrono::milliseconds(cfg_.get_scan_period_ms()),
        std::bind(&FakeLidarNode::publish_scan_, this));

    frame_brodcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped transform;
    transform.child_frame_id = "laser_frame";
    transform.header.frame_id = "robot";
    transform.header.stamp = this->now();
    frame_brodcaster_->sendTransform(transform);

}

void FakeLidarNode::publish_scan_()
{
    CyclicObstacle obstacle(center_, cfg_.sigma, cfg_.sample_count);
    std::vector<float> ranges(cfg_.sample_count);
    for(int i=0; i<cfg_.sample_count; i++)
    {
      ranges[i] = cfg_.get_scaled_sample(obstacle[i]);
    }
    scan_.ranges = ranges;
    scan_.header.stamp = this->now();
    laser_pub_->publish(scan_);
    center_++;
    if(center_ >= cfg_.sample_count)
    {
        center_ = 0;
    }
}

void FakeLidarNode::prepare_msg_config_()
{
    scan_.angle_min = cfg_.angle.first;
    scan_.angle_max = cfg_.angle.second;
    scan_.angle_increment = cfg_.get_scan_step();
    scan_.range_min = cfg_.range.first;
    scan_.range_max = cfg_.range.second;
    scan_.scan_time = cfg_.get_scan_period_ms()/1000;
    scan_.header.frame_id = "laser_frame";
}

//------------------------------------------------------------------------------
//                             FAKE LIDAR NODE
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeLidarNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}