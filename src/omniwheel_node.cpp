#include <ros/ros.h>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int16.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>


ros::Publisher motorSpeedPub;
ros::Publisher motorWrenchPub;
tf2_ros::Buffer tfBuffer;

void controlCallback(const geometry_msgs::Twist &twist);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "omniwheel");
    ros::NodeHandle nh;

    ros::Subscriber controlSub = nh.subscribe("cmd_vel", 512, controlCallback);

    /*ros::Publisher*/ motorSpeedPub = nh.advertise<std_msgs::Int16>("motor_speed", 512);
    // motorSpeedPub_ptr = &motorSpeedPub;

    /*ros::Publisher*/ motorWrenchPub = nh.advertise<geometry_msgs::WrenchStamped>("motor_wrench", 1);
    // motorWrenchPub_ptr = &motorWrenchPub;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin();

    return 0;
}

void controlCallback(const geometry_msgs::Twist &twist)
{
    double x = twist.linear.x;
    double y = twist.linear.y;
    double turn = twist.angular.z;

    double wheel_x = 0.f;
    double wheel_y = 0.f;
    double wheel_angle;

    int wheel_id;
    ros::param::get("~wheel_id", wheel_id);
    try {
        // get wheel position from transform
        geometry_msgs::TransformStamped wheelTransform = tfBuffer.lookupTransform("base_link", "wheel"+std::to_string(wheel_id), ros::Time(0));

        wheel_x = wheelTransform.transform.translation.x;
        wheel_y = wheelTransform.transform.translation.y;

        // ROS_DEBUG("Wheel %d X = %f Y = %f", wheel_id, wheel_x, wheel_y);

        // get wheel angle on Z axis from transform
        tf2::Quaternion quat(wheelTransform.transform.rotation.x, wheelTransform.transform.rotation.y, wheelTransform.transform.rotation.z, wheelTransform.transform.rotation.w);
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, wheel_angle);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        // stop motor on exception
        x = 0.f;
        y = 0.f;
        turn = 0.f;
    }

    const double turn_scale = 8.f;

    // add rotation to speed vector
    // twist           + rotate(wheel)              - wheel
    double rotated_x = cos(turn)*wheel_x*turn_scale - sin(turn)*wheel_y*turn_scale;
    double rotated_y = sin(turn)*wheel_x*turn_scale + cos(turn)*wheel_y*turn_scale;
    ROS_DEBUG("X = %f Y = %f", wheel_x, wheel_y);
    ROS_DEBUG("Xr = %f Yr = %f", rotated_x, rotated_y);
    x = x + rotated_x - wheel_x*turn_scale;
    y = y + rotated_y - wheel_y*turn_scale;

    ROS_DEBUG("Xt = %f Yt = %f", x, y);

    // align wheel plane with Y axis and project speed vector to new wheel plane
    const double wheel_speed = x * sin(-wheel_angle) + y * cos(-wheel_angle);

    geometry_msgs::WrenchStamped wrench;
    wrench.header.frame_id = "wheel"+std::to_string(wheel_id);
    wrench.header.stamp = ros::Time::now();

    wrench.wrench.force.y = wheel_speed;
    wrench.wrench.torque.x = wheel_speed;

    motorWrenchPub.publish(wrench);

    std_msgs::Int16 motor_control;
    motor_control.data = -wheel_speed * 100;
    motorSpeedPub.publish(motor_control);
}
