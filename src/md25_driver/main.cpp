#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <cmath>
#include <md25_driver/md25.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
// Based on https://docs.ros.org/en/ros2_packages/rolling/api/std_msgs/interfaces/msg/MultiArrayDimension.html
// These messages a depreciated. See 
// # This was originally provided as an example message.
// # It is deprecated as of Foxy
// # It is recommended to create your own semantically meaningful message.
// # However if you would like to continue using this please use the equivalent in example_msgs.
// #include <std_msgs/msg/multi_arrayLayout.h>
// #include <std_msgs/msg/int16_multiArray.h>
// #include <std_msgs/msg/int32_multiArray.h>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
// #include <diagnostic_msgs/msg/diagnostic_array.hpp>
// #include <diagnostic_msgs/msg/diagnostic_status.hpp>

// #include <diagnostic_msgs/msg/key_value.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MD25MotorDriverROSWrapper : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_twist_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_publisher_;  
  
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr current_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr motor_status_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motor_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoders_server_;
  
  rclcpp::TimerBase::SharedPtr current_speed_timer_;
  rclcpp::TimerBase::SharedPtr motor_status_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr pid_timer_;

  rclcpp::Time  last_time;             // Last time when the calculation has been performed

  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::string base_link =  "/base_link";
  std::string odom =  "/odom";
// char base_link[] = "/base_link";
// char odom[] = "/odom";

  //-----------------------------------------------
  const double PI = 3.14159265358979323846;
  double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  double publish_motor_encoders_frequency_;
  double publish_odom_frequency_;
  double pid_frequency_;
  bool debug_mode_ = false;
  bool enable_speed_ = false;
  bool enable_odom_ = false;
  bool enable_pid_ = false;
  
  const bool enable_twist_default = true;
  bool enable_twist_ = enable_twist_default;
  bool enable_status_ = false;
  double wheelDiameter = 0.210;
  double wheelTrack = 0.345;
  int cpr = 1080;
  double wheelCircum =  PI * wheelDiameter; //0.65973
  double ticksPerMeter = (cpr / wheelCircum); //1637.0222
  double max_linear_x = 1.0;
  double max_angular_z = 1.0;
  bool has4Motor = true;
  
  //   Mode Register
  // The mode register selects which mode of operation and I2C data input type the user requires. The options being:
  // 0,    (default setting) If a value of 0 is written to the mode register then the meaning of the speed registers is literal speeds in the range of 0 (full reverse)  128 (stop)   255 (full forward).
  // 1,    Mode 1 is similar to mode 0, except that the speed registers are interpreted as signed values. The meaning of the speed registers is literal speeds in the range of -128 (full reverse)   0 (Stop)   127 (full forward).
  // 2,    Writing a value of  2 to the mode register will make Speed1 control both motors speed, and Speed2 becomes the turn value. 
  // Data is in the range of 0 (full reverse)  128 (stop)  255 (full  forward).
  // 3,    Mode 3 is similar to mode 2, except that the speed registers are interpreted as signed values. 
  // Data is in the range of -128  (full reverse)  0 (stop)   127 (full forward)
  int motor_mode_ = 1;
  int max_speed_ = 100;
  bool moving = false;
  // Acceleration Rate 
  // If you require a controlled acceleration period for the attached motors to reach there ultimate speed, the MD25 has a register to provide this. 
  // It works by using a value into the acceleration register and incrementing the power by that value. Changing between the current speed of the motors 
  // and the new speed (from speed 1 and 2 registers). So if the motors were traveling at full speed in the forward direction (255) and were instructed 
  // to move at full speed in reverse (0), there would be 255 steps with an acceleration register value of 1, but 128 for a value of 2. 
  // The default acceleration value is 5, meaning the speed is changed from full forward to full reverse in 1.25 seconds. The register will accept values 
  // of 1 up to 10 which equates to a period of only 0.65 seconds to travel from full speed in one direction to full speed in the opposite direction.
  // See https://www.robot-electronics.co.uk/htm/md25i2c.htm
  int acceleration_rate = 3;

  double width_roboter  = 0.33;    // This is the distance from one wheel to the other in meter
  // // double distance_front_wheel  = 0.36;    // This is the distance from center of roboter to the front wheel in meter
  // //   double distance_rear_wheel  = 0.36;    // This is the distance from one wheel to the other in meter
  double distance_gravity_axis  = 0.26;    // This is the distance from center of roboter to the front wheel in meter
  double distance_gravity_wheel = 0.20;    // This is the distance from center of roboter to the rear wheel in meter  
  double velocity_front_right = 0.0;     // velocity of the right wheel in m / s 
  double velocity_front_left  = 0.0;     // velocity of the right wheel in m / s 
  double velocity_rear_right = 0.0;     // velocity of the right wheel in m / s 
  double velocity_rear_left  = 0.0;     // velocity of the right wheel in m / s 
  double front_right_encoder_old = 0.0;	// Last processed encoder value right 
  double front_left_encoder_old = 0.0;	// Last processed encoder value left
  double rear_right_encoder_old = 0.0;	// Last processed encoder value right 
  double rear_left_encoder_old = 0.0;		// Last processed encoder value left
  double front_right_encoder = 0.0;		 // Last encoder value right 
  double front_left_encoder = 0.0;		 // Last encoder value left
  double rear_right_encoder = 0.0;		 // Last encoder value right 
  double rear_left_encoder = 0.0;			 // Last encoder value left

  double x = 0.0;					 // current x position
  double y = 0.0;					 // current y position
  double theta = 0.0;				 // current rotation
  geometry_msgs::msg::Quaternion odom_quat;
  tf2::Quaternion quat;


double ticks_per_meter = 1146.131;    // conversion factor from meter to ticks 
                                 // EMG30 has Encoder counts per output shaft turn 360 
                                 // Wheel is 100mm diameter.
                                 // circumference is then 2 * PI * Radius ==> 314,1 mm per turn
                                 // 1000 mm / 314,1 mm * 360 ticks per round
double maximumSpeed = 0.9423;	 // This is the maximum speed for the EMG30 motor which is 
                                 // is 216 Rounds Per Minute. 
                                 // This makes 3 Rounds per second maximum speed
                                 // Which is 0,9423 m/s


//-------------------------------------------------
double Kp = 2.0;
double Ki = 0.5;
double Kd = 0.0;
double Ko = 10.0;

//-------------------------------------------------
/* Setpoint Info For a Motor */
typedef struct {
  double TargetTicksPerFrame = 0.0;  // target speed in ticks per frame
  long Encoder =0;                  // encoder count
  long PrevEnc =0;                  // last encoder count
  long PrevErr = 0;                   // last error
  long output =0;                    // last motor setting
  int PrevInput =0;                // last input
  int ITerm =0;                    //integrated term
}
SetPointInfo;
SetPointInfo leftPID, rightPID;

//-------------------------------------------------
// /* Odometry Data */
typedef struct {
  rclcpp::Time OdomStamp;                // last ROS time odometry was calculated
  rclcpp::Time PrevOdomStamp;                // last ROS time odometry was calculated
  long LeftEnc =0;              // last left encoder reading used for odometry
  long RightEnc =0;             // last right encoder reading used for odometry
}
OdomInfo;
OdomInfo odomInfo;

//-------------------------------------------------
/* Current Pose */
typedef struct {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double xVel = 0.0;
  double yVel = 0.0;
  double thetaVel = 0.0;
} Pose;

Pose pose;
//---------------------------------------
public:
  std::unique_ptr<md25_driver> motor;
//---------------------------------------
 MD25MotorDriverROSWrapper(const rclcpp::NodeOptions & options) : Node("bus_master", options) {

    odomInfo.OdomStamp = this->get_clock()->now();

    this->declare_parameter<double>("publish_current_speed_frequency", 10.0);
    this->declare_parameter<double>("publish_motor_status_frequency", 10.0);
    this->declare_parameter<double>("publish_odom_frequency", 10.0);
    this->declare_parameter<bool>("enable_odom", false);    
    this->declare_parameter<bool>("enable_status", false);       
    this->declare_parameter<bool>("enable_twist", enable_twist_default);    
   
    setParams();

    //------------------------------------------
    motor.reset(new md25_driver("/dev/i2c-1"));
    bool setup = motor->setup(this->get_logger ());
    bool mode1 = motor->setMode(this->get_logger (), motor->getDeviceIdFront(), motor_mode_);
    bool accel = motor->setAccelerationRate(this->get_logger (), motor->getDeviceIdFront(), acceleration_rate);
    
    if(!setup){
      RCLCPP_ERROR(this->get_logger(), "failed to setup motor driver!");
    }

    if(!mode1){
      RCLCPP_ERROR(this->get_logger(), "failed to set motor to mode %d!",motor_mode_);
    }else{
      RCLCPP_INFO(this->get_logger(),"MD25 Motor Mode set to %d",motor_mode_);
    }

    if(!accel){
      RCLCPP_ERROR(this->get_logger(), "failed to set motor to acceleration rate %d!",acceleration_rate);
    }else{
      RCLCPP_INFO(this->get_logger(),"MD25 Motor acceleration rate %d",acceleration_rate);
    }

    bool mode2 = motor->setMode(this->get_logger (), motor->getDeviceIdRear(), motor_mode_);
    bool accel2 = motor->setAccelerationRate(this->get_logger (), motor->getDeviceIdRear(), acceleration_rate);
    
    if(!mode2){
      RCLCPP_ERROR(this->get_logger(), "failed to set motor to mode %d!",motor_mode_);
    }else{
      RCLCPP_INFO(this->get_logger(),"MD25 Motor Mode set to %d",motor_mode_);
    }

    if(!accel2){
      RCLCPP_ERROR(this->get_logger(), "failed to set motor to acceleration rate %d!",acceleration_rate);
    }else{
      RCLCPP_INFO(this->get_logger(),"MD25 Motor acceleration rate %d",acceleration_rate);
    }

    if(!resetAll()){
      RCLCPP_ERROR(this->get_logger(), "failed to reset encoders!");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    //------------------------------------------
    stop_motor_server_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_motors", std::bind(&MD25MotorDriverROSWrapper::stopMotorCallback, this, std::placeholders::_1, std::placeholders::_2)); 

    reset_encoders_server_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_encoders",
        std::bind(&MD25MotorDriverROSWrapper::callbackReset, this, std::placeholders::_1, std::placeholders::_2)); 

    if (enable_twist_) {
        motor_twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 2, std::bind(&MD25MotorDriverROSWrapper::twistToMotors, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "MD25 Motor cmd_vel Subscribe Enabled");
    }

    if (enable_odom_) {
        odom_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_odom_frequency_),
            std::bind(&MD25MotorDriverROSWrapper::publishOdom, this));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        RCLCPP_INFO(this->get_logger(), "MD25 Odom Publish Enabled");
    }

    if (enable_speed_) {
        current_speed_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_current_speed_frequency_),
        std::bind(&MD25MotorDriverROSWrapper::publishCurrentSpeed, this)
        );
        current_speed_publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("current_speed", 10);
        RCLCPP_INFO(this->get_logger(), "MD25 Motor Speed Publish Enabled");
    }

    if (enable_status_) {
        motor_status_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_motor_status_frequency_),
            std::bind(&MD25MotorDriverROSWrapper::publishMotorStatus, this));
        motor_status_publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("motor_status", 1);
        RCLCPP_INFO(this->get_logger(), "MD25 Motor Status Publish Enabled");
    }

    // TODO   
    // if(enable_pid_){
    //   pid_timer_ = nh->createTimer(ros::Duration(1.0/pid_frequency_), &MD25MotorDriverROSWrapper::UpdatePID,this);
    //   RCLCPP_INFO(this->get_logger(),"MD25 PID Enabled");
    // }
  }

//---------------------------------------
/* Set Incoming Parameters to Variables or Defaults */
void setParams() {

   if(!this->get_parameter("enable_speed",enable_speed_)){
      enable_speed_ = false;
    }
    if (!this->get_parameter("publish_current_speed_frequency", publish_current_speed_frequency_)) {
      publish_current_speed_frequency_ = 0.0;
      enable_speed_ = false;
    }else{
      if(publish_current_speed_frequency_ == 0.0){
        enable_speed_ = false;
      }else{
        enable_speed_ = true;
      }
    }

    if(!this->get_parameter("enable_status",enable_status_)){
      enable_status_ = false;
    }

    if (!this->get_parameter("publish_motor_status_frequency", publish_motor_status_frequency_)) {
      publish_motor_status_frequency_ = 0.0;
      enable_status_ = false;
    }else{
      if(publish_motor_status_frequency_ == 0.0){
        enable_status_ = false;
      }else{
        enable_status_ = true;
      }
    }

    if(!this->get_parameter("enable_twist",enable_twist_)){
      enable_twist_ = enable_twist_default;
    }

    if(!this->get_parameter("enable_odom",enable_odom_)){
      enable_odom_ = false;      
    }
    if(!this->get_parameter("acceleration_rate",acceleration_rate)){
      acceleration_rate = 3;
    }

    if(!this->get_parameter("publish_odom_frequency",publish_odom_frequency_)){
      publish_odom_frequency_ = 10.0;
      enable_odom_ = false;
    }else{
      if(publish_odom_frequency_ == 0.0){
        enable_odom_ = false;
      }else{
        enable_odom_ = true;
      }
    }
    // if(!ros::param::get("~motor_mode",motor_mode_)){
    //   motor_mode_ = 1;
    // }
    // if(!ros::param::get("~wheel_diameter",wheelDiameter)){
    //   wheelDiameter = 0.210;
    // }
    // if(!ros::param::get("~wheel_track",wheelTrack)){
    //   wheelTrack = 0.355;
    // }
    // if(!ros::param::get("~encoder_clicks",cpr)){
    //   cpr = 1080;
    // }
    // if(!ros::param::get("~max_speed",max_speed_)){
    //   max_speed_ = 100;
    // }
    // if(!ros::param::get("~pid_p",Kp)){
    //   Kp = 2.0;
    // }
    // if(!ros::param::get("~pid_i",Ki)){
    //   Ki = 0.5;
    // }
    // if(!ros::param::get("~pid_d",Kd)){
    //   Kd = 0.0;
    // }
    // if(!ros::param::get("~pid_o",Ko)){
    //   Ko = 50.0; //4.0 * pid_frequency_;
    // }
    // if(!ros::param::get("~pid_frequency",pid_frequency_)){
    //   pid_frequency_ = 10.0;
    // }
    // if(!ros::param::get("~debug_mode",debug_mode_)){
    //   debug_mode_ = false;
    // }
    // if(!ros::param::get("~enable_pid",enable_pid_)){
    //   enable_pid_ = false;
    // }

    RCLCPP_INFO(this->get_logger(),"MD25 Parameters Set");
}

//---------------------------------------
/* Handle reset_encoders service message */
void callbackReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
    (void) request;
    motor->resetEncoders(this->get_logger (), motor->getDeviceIdFront());   
    if (has4Motor)
    {
      motor->resetEncoders(this->get_logger (), motor->getDeviceIdRear());
    }
    response->success = true;
    response->message = "Encoders Reset";    
    RCLCPP_INFO(this->get_logger(),"Encoders Reset");
 }
//---------------------------------------
/* Handle stop_motors service message */
void stopMotorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
     (void) request;
    stop();
    response->success = true;
    response->message = "Stopped Motors";
    RCLCPP_INFO(this->get_logger(),"Motor Stop");
}


//---------------------------------------
/* optional publish current_speed (motor values) */ 
void publishCurrentSpeed(){
  int speed_l;
  int speed_r;
  std_msgs::msg::ByteMultiArray barry;
  std::tie(speed_l, speed_r) = motor->getMotorsSpeed(this->get_logger(), motor->getDeviceIdFront());
  barry.data.clear();
  barry.data.push_back(speed_l);
  barry.data.push_back(speed_r);
  if (has4Motor)
  {
    std::tie(speed_l, speed_r) = motor->getMotorsSpeed(this->get_logger(), motor->getDeviceIdRear());
    barry.data.push_back(speed_l);
    barry.data.push_back(speed_r);
  }
  current_speed_publisher_->publish(barry);
 }

// // ROS 1
// // //---------------------------------------
// // /* optional publish motor_encoders (raw values) */
// // // void publishEncoders(const ros::TimerEvent &event){
// // //   int ticks_l;
// // //   int ticks_r;
// // //   std::tie(ticks_l, ticks_r) = motor->readEncoders();
// // //   //std::pair<int,int> ticks = motor->readEncoders();
// // //   std_msgs::ByteMultiArray barry;
// // //   barry.data.clear();
// // //   barry.data.push_back(-ticks_l);
// // //   barry.data.push_back(-ticks_r);
// // //   motor_encoders_publisher_.publish(barry);
// // //  }

//---------------------------------------
/* optional publish motor currents and battery voltage */
void publishMotorStatus(){
  int curr_l;
  int curr_r;
  std::tie(curr_l, curr_r) = motor->readEncoders(this->get_logger(), motor->getDeviceIdFront());
  std_msgs::msg::ByteMultiArray barry;
  barry.data.clear();
  barry.data.push_back(curr_l);
  barry.data.push_back(curr_r);
 if (has4Motor)
  {
    std::tie(curr_l, curr_r) = motor->readEncoders(this->get_logger(), motor->getDeviceIdRear());
    barry.data.push_back(curr_l);
    barry.data.push_back(curr_r);
  }  
  motor_status_publisher_->publish(barry);
 }

//---------------------------------------
/* Direct Stop Both Motors*/
void stop(){
    motor->stopMotors(this->get_logger (), motor->getDeviceIdFront());
    if (has4Motor){
      motor->stopMotors(this->get_logger (), motor->getDeviceIdRear());
    }
 }

//---------------------------------------
/* Shutdown routine */
void shutdown(){
   stop();
 }

//---------------------------------------
/* Calculate Pose from Left/Right Encoders and Velocities */
void calculatePose(int dL,int dR,double dt){
  double leftTravel = dL / ticksPerMeter;
  double rightTravel = dR / ticksPerMeter;
  double deltaTravel = (rightTravel + leftTravel)/2;
  double deltaTheta = (rightTravel - leftTravel)/wheelTrack;
  double vx = deltaTravel / dt;
  double vr = deltaTheta / dt;
  
  if(deltaTravel !=0.0){
    double x = cos(deltaTheta) * deltaTravel;
    double y = -sin(deltaTheta) * deltaTravel;
    pose.x = pose.x + (cos(pose.theta) * x - sin(pose.theta) * y);
    pose.y = pose.y + (sin(pose.theta) * x + cos(pose.theta) * y);
  }
  if (deltaTheta !=0.0){
    pose.theta = pose.theta + deltaTheta;
  }

 pose.yVel = 0.0;
 if(dt == 0.0){
  pose.xVel = 0.0;
  pose.thetaVel = 0.0;
 }else{
  pose.xVel = vx;
  pose.thetaVel = vr;
 }
}

void read_encoder() {                  // Function to read and display value of encoder 2 as a long
  int ticks_l;
  int ticks_r;

  // Encode 1 from MD25 is here left, encodeer 2 from MD25 is here right
  std::tie(ticks_l, ticks_r) = motor->readEncoders(this->get_logger (), motor->getDeviceIdFront());
  front_right_encoder = (double) ticks_l;			// Last encoder value right 
  front_left_encoder	= (double) ticks_r;			// Last encoder value left
  std::tie(ticks_l, ticks_r) = motor->readEncoders(this->get_logger (), motor->getDeviceIdRear());
  rear_right_encoder	= (double) ticks_l;			// Last encoder value right 
  rear_left_encoder	  = (double) ticks_r;			// Last encoder value left
}


void publishOdom() {


  double dxy = 0.0;
  double dth = 0.0;
  // ros::Time current_time = nh.now();
  rclcpp::Time current_time = this->get_clock()->now();
  double dt;
  double distance_left = 0.0;		 // calculated travel distance right 
  double distance_right = 0.0;	 // calculated travel distance left
  geometry_msgs::msg::TransformStamped t;

  read_encoder ();

  dt =  (current_time.seconds() - last_time.seconds());
  last_time = current_time;


  // calculate odomety
  distance_left = (front_left_encoder - front_left_encoder_old) / ticks_per_meter;
  distance_right = (front_right_encoder - front_right_encoder_old) / ticks_per_meter;

  front_left_encoder_old = front_left_encoder;
  front_right_encoder_old = front_right_encoder;
  rear_left_encoder_old = rear_left_encoder;
  rear_right_encoder_old = rear_right_encoder;

  dxy = (distance_left + distance_right) / 2.0;
  dth = (distance_right - distance_left) / width_roboter;

  if(dxy != 0){
    x += dxy * cosf(theta);
    y += dxy * sinf(theta);
  }	

  if(dth != 0){
    theta += dth;
  }

  double velxy = dxy / dt;
  double velth = dth / dt;


  t.header.stamp = this->get_clock()->now();
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  // odom_quat = tf::createQuaternionFromYaw(theta);
  quat.setRPY(0,0, theta);

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation = tf2::toMsg (quat);
  transform_broadcaster_->sendTransform(t);

  //  next, we'll publish the odometry message over ROS 2
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //  set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //  set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = velxy;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = velth;

  //  publish the message
  odom_publisher_->publish(odom);

        // rclcpp::Time currentTime = this->get_clock()->now();
        // odomInfo.OdomStamp = currentTime;

        // int ticks_l;
        // int ticks_r;
        // std::tie(ticks_l, ticks_r) = motor->readEncoders(this->get_logger (), motor->getDeviceIdFront());

        // leftPID.Encoder = ticks_l;
        // rightPID.Encoder = ticks_r;

        // int deltaLeft = leftPID.Encoder - odomInfo.LeftEnc;
        // int deltaRight = rightPID.Encoder - odomInfo.RightEnc;

        // odomInfo.LeftEnc = leftPID.Encoder;
        // odomInfo.RightEnc = rightPID.Encoder;

        // rclcpp::Duration dt = odomInfo.OdomStamp - odomInfo.PrevOdomStamp;
        // odomInfo.PrevOdomStamp = currentTime;

        // calculatePose(deltaLeft, deltaRight, dt.seconds());


        // // CHECK This transform 
        // // See https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html

        // tf2::Quaternion q;
        // q.setRPY(0.0, 0.0, pose.theta);
        // geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);
        // // ROS1 geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, pose.theta));

        // // Transform message
        // geometry_msgs::msg::TransformStamped odom_trans;
        // odom_trans.header.stamp = currentTime;
        // odom_trans.header.frame_id = "odom";
        // odom_trans.child_frame_id = "base_link";

        // odom_trans.transform.translation.x = pose.x;
        // odom_trans.transform.translation.y = pose.y;
        // odom_trans.transform.translation.z = 0.0;
        // odom_trans.transform.rotation = odom_quat;

        // transform_broadcaster_->sendTransform(odom_trans);

        // // Odometry message
        // nav_msgs::msg::Odometry odom;
        // odom.header.stamp = currentTime;
        // odom.header.frame_id = "odom";
        // odom.child_frame_id = "base_link";

        // odom.pose.pose.position.x = pose.x;
        // odom.pose.pose.position.y = pose.y;
        // odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odom_quat;

        // odom.twist.twist.linear.x = pose.xVel;
        // odom.twist.twist.linear.y = pose.yVel;
        // odom.twist.twist.linear.z = 0.0;
        // odom.twist.twist.angular.x = 0.0;
        // odom.twist.twist.angular.y = 0.0;
        // odom.twist.twist.angular.z = pose.thetaVel;
        // odom_publisher_->publish(odom);

        if (debug_mode_) {
            RCLCPP_INFO(this->get_logger(),
                        "Odom: dL=%f, dR=%f - dt:%f (x=%f, y=%f - t:%f)",
                        distance_left, distance_right, dt, pose.x, pose.y, pose.theta);
        }
    }
 
// // //---------------------------------------
// // /* Publish Odometry and tf */
// // void publishOdom(const ros::TimerEvent &event){
// //   ros::Time currentTime = ros::Time::now();
// //   odomInfo.OdomStamp = currentTime;
// //   int ticks_l;
// //   int ticks_r;
// //   std::tie(ticks_l, ticks_r) = motor->readEncoders();
// //   leftPID.Encoder = ticks_l;
// //   rightPID.Encoder = ticks_r;

// //   int deltaLeft = leftPID.Encoder - odomInfo.LeftEnc;
// //   int deltaRight = rightPID.Encoder - odomInfo.RightEnc;
// //   odomInfo.LeftEnc = leftPID.Encoder;
// //   odomInfo.RightEnc = rightPID.Encoder;
// //   ros::Duration dt = odomInfo.OdomStamp - odomInfo.PrevOdomStamp;
// //   odomInfo.PrevOdomStamp = currentTime;
// //   calculatePose(deltaLeft,deltaRight,dt.toSec());

// //   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

// //   //Transform message
// //   geometry_msgs::TransformStamped odom_trans;
// //   odom_trans.header.stamp = currentTime; //odomInfo.OdomStamp;
// //   odom_trans.header.frame_id = "odom";
// //   odom_trans.child_frame_id = "base_link";
// //   odom_trans.transform.translation.x = pose.x;
// //   odom_trans.transform.translation.y = pose.y;
// //   odom_trans.transform.translation.z = 0.0;
// //   odom_trans.transform.rotation = odom_quat;
// //   transform_broadcaster_.sendTransform(odom_trans);

// //   //Odometry message
// //   nav_msgs::Odometry odom;
// //   odom.header.stamp = currentTime; //odomInfo.OdomStamp;
// //   odom.header.frame_id = "odom";
// //   odom.child_frame_id = "base_link";
// //   odom.pose.pose.position.x = pose.x;
// //   odom.pose.pose.position.y = pose.y;
// //   odom.pose.pose.position.z = 0.0;
// //   odom.pose.pose.orientation = odom_quat;
// //   odom.twist.twist.linear.x = pose.xVel;
// //   odom.twist.twist.linear.y = pose.yVel;
// //   odom.twist.twist.linear.z = 0.0;
// //   odom.twist.twist.angular.x = 0.0;
// //   odom.twist.twist.angular.y = 0.0;
// //   odom.twist.twist.angular.z = pose.thetaVel;
// //   odom_publisher_.publish(odom);

// //   if(debug_mode_){
// //     RCLCPP_INFO(this->get_logger(),"Odom: dL=%d, dR=%d - dt:%f (x=%f, y=%f - t:%f)", deltaLeft, deltaRight,dt.toSec(), pose.x, pose.y,pose.theta);
// //   }
// // }
//--------------------------------------------------------
/* Incoming cmd_vel message to Motor commands */
void twistToMotors(const geometry_msgs::msg::Twist &msg){

  geometry_msgs::msg::Twist twist = msg;	
  double vel_x = twist.linear.x;
  double vel_y = twist.linear.y;
  double vel_th = twist.angular.z;
  velocity_front_left   =  vel_x - vel_y - vel_th * (distance_gravity_axis + distance_gravity_wheel);
  velocity_front_right  =  vel_x + vel_y + vel_th * (distance_gravity_axis + distance_gravity_wheel);
  velocity_rear_left    =  vel_x + vel_y - vel_th * (distance_gravity_axis + distance_gravity_wheel);
  velocity_rear_right   =  vel_x - vel_y + vel_th * (distance_gravity_axis + distance_gravity_wheel);

  int front_right = convertVelocityToMotorSpeed(velocity_front_right);
  int front_left = convertVelocityToMotorSpeed(velocity_front_left);
  int rear_right = convertVelocityToMotorSpeed(velocity_rear_right);
  int rear_left = convertVelocityToMotorSpeed(velocity_rear_left);

  motor->setMotorsSpeed (this->get_logger(), motor->getDeviceIdFront(), front_left, front_right);
  motor->setMotorsSpeed (this->get_logger(), motor->getDeviceIdRear(), rear_left, rear_right); // ??
  // setMotorSpeed(FRONT, FRONTLEFT, front_left);
  // setMotorSpeed(FRONT, FRONTRIGHT, front_right);
  // setMotorSpeed(REAR, REARLEFT, rear_left);
  // setMotorSpeed(REAR, REARRIGHT, rear_right);


  // double x = msg.linear.x;
  // double th = msg.angular.z;
  // double spd_left,spd_right;
  // if(x == 0.0 && th == 0.0){
  //   moving = false;
  //   motor->stopMotors(this->get_logger (), motor->getDeviceIdFront());
  //   return;
  // }
  // moving = true;
  // double velDiff = (wheelTrack * th) /2;
  // spd_left = ((x - velDiff) / (wheelDiameter/2.0));
  // spd_right = ((x + velDiff) / (wheelDiameter/2.0));
  
  // if(enable_pid_){
  //   leftPID.TargetTicksPerFrame = SpeedToTicks(spd_left);
  //   rightPID.TargetTicksPerFrame = SpeedToTicks(spd_right);
  //   if(debug_mode_){
  //     RCLCPP_INFO(this->get_logger(),"twist: L=%f, R=%f - ttL:%f ttR:%f", spd_left, spd_right,leftPID.TargetTicksPerFrame,rightPID.TargetTicksPerFrame);
  //   }
  // }
}


// Convert from m/s speed into the command values from the 
// Motor and the number of the speed

int convertVelocityToMotorSpeed (double speed)
{
    if ((speed < 0.0001) && (speed > -0.0001))
    {
        //This is nearly zero so stop
        return 0;
    }
    if (speed >= maximumSpeed)
    {
        return 128;
    }
    if (speed <= - maximumSpeed)
    {
        return -128;
    }
    // Maximum speed is divided by the maximum speed to get the 
    // steps in which the speed can be controlled. 
    // Intended speed divided by these steps gives the number for the MD25
    int emg30Speed = (speed / (maximumSpeed / 127.0));
    if (emg30Speed  > 128)
    {
        emg30Speed = 128;
    }
    if (emg30Speed  < -128)
    {
        emg30Speed = -128;
    }
    return emg30Speed ;
}

// //------------------------------------------------------------------
// //https://github.com/KristofRobot/ros_arduino_bridge/commit/cf9d223969d1be2d6d954f8cbaa67a331c8a2793
// /* PID routine to compute the next motor commands */
// void doPID(SetPointInfo * p) {
//   long Perror;
//   long output;
//   int input;
//   input = p->Encoder - p->PrevEnc;
//   Perror = p->TargetTicksPerFrame - input;
//   // Derivative error is the delta Perror
//   output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm)/Ko;
//   p->PrevEnc = p->Encoder; 
//   //output += p->output; // cumulative don't work
//   if (output >= max_speed_)
//     output = max_speed_;
//   else if (output <= -max_speed_)
//     output = -max_speed_;
//   else
//     p->ITerm += Ki * Perror;
//   p->output = output;
//   p->PrevInput = input;
// }
//---------------------------------------
/* Clear PID Values and variables */
bool clearPID(){
  moving = false;
  
  leftPID.PrevErr = 0;
  leftPID.output = 0;
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;
  rightPID.PrevErr = 0;
  rightPID.output = 0;
  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
  leftPID.PrevEnc = leftPID.Encoder;  
  rightPID.PrevEnc = rightPID.Encoder;
  odomInfo.LeftEnc = leftPID.Encoder;
  odomInfo.RightEnc = rightPID.Encoder;
  if(debug_mode_){
    RCLCPP_INFO(this->get_logger(),"MD25 PID Cleared");
  }
  return true;
}
//---------------------------------------
bool resetAll(){
  bool resetted  = motor->resetEncoders(this->get_logger (), motor->getDeviceIdFront());

  if (has4Motor)
  {
    resetted  &= motor->resetEncoders(this->get_logger (), motor->getDeviceIdRear());
  }

  RCLCPP_INFO(this->get_logger(),"MD25 Reset ALL");
  // int ticks_l;
  // int ticks_r;
  // ros::Duration(0.006).sleep();
  // std::tie(ticks_l, ticks_r) = motor->readEncoders();
  leftPID.Encoder = 0;
  rightPID.Encoder = 0;
  clearPID();
  return resetted;
}

// ROS 1
// //---------------------------------------------------------------
// /* Read the encoder values and call the PID routine */
// void UpdatePID(const ros::TimerEvent &event) {
//   // int ticks_l;
//   // int ticks_r;
//   // std::tie(ticks_l, ticks_r) = motor->readEncoders();
//   // leftPID.Encoder = ticks_l;
//   // rightPID.Encoder = ticks_r;
  
//   doPID(&leftPID);
//   doPID(&rightPID);
//   if (!moving){
//     if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) clearPID();
//     return;
//   }
//   motor->writeSpeed(leftPID.output, rightPID.output);
//   if(debug_mode_){
//     RCLCPP_INFO(this->get_logger(),"updatePID:  PIDL:%ld PIDR:%ld - EL:%ld ER:%ld", leftPID.output,rightPID.output,leftPID.PrevErr,rightPID.PrevErr);
//   }
// }

//---------------------------------------
double MPStoMotorSpeed(double ms){
  if(ms==0.0) return 0.0;
  double maxms = 1.0;
  if(ms>maxms)ms=maxms;
  if(ms<-maxms)ms=-maxms;
  return (maxms / 127) * ms;
}
//---------------------------------------
double SpeedToTicks(double v) {
  if(v==0.0) return 0.0;
  double ticks = (v * cpr / (pid_frequency_ * PI * wheelDiameter));
  if(isnan(ticks)){return 0.0;}
  return ticks;
}
//---------------------------------------
/* return current time in milliseconds */
long Millis(){
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch()).count();
 }

}; // end class
//---------------------------------------
/* Main Function */
int main(int argc,char **argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;   
    auto node = std::make_shared<MD25MotorDriverROSWrapper>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  
    node.get()->shutdown();
    rclcpp::shutdown();

  // // ROS 1
  // // ros::AsyncSpinner spinner(4);
  // // spinner.start();
  // // MD25MotorDriverROSWrapper motor_wrapper(&nh);
  RCLCPP_INFO(node->get_logger(),"MD25 Motor Driver v0.0.1 Started");
  // // ros::waitForShutdown();
  // // motor_wrapper.shutdown();
  return 0;
 }