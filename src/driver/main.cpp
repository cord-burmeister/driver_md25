#include <map>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <cmath>
#include <i2c_bus.hpp>
#include <driver/md25.hpp>
#include <driver/hiwonder.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
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

  //-----------------------------------------------
  // configuration of the logic which is activated in the node
  double publish_current_speed_frequency_;
  double publish_motor_status_frequency_;
  double publish_motor_encoders_frequency_;
  double publish_odom_frequency_;
  bool enable_speed_ = false;
  bool enable_odom_ = false;  
  const bool enable_twist_default = true;
  bool enable_twist_ = enable_twist_default;
  bool enable_status_ = false;
  bool debug_mode_ = true;


//-----------------------------------------------
// configuration parameter of the physical robot
  
  const double PI = 3.14159265358979323846;
  double wheel_diameter = 0.10;


  
  double max_linear_x = 1.0;
  double max_angular_z = 1.0;
  
  double  max_speed_ = 100.0;

  double width_robot  = 0.33;    // This is the distance from one wheel to the other in meter
  double distance_front_wheel  = 0.26;    // This is the distance from center of robot to the front wheel in meter
  double distance_rear_wheel = 0.20;    // This is the distance from center of robot to the rear wheel in meter  

  //-----------------------------------------------
  // internal calcuation values for the odometry and speed control
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

  //-----------------------------------------------
  // Motor unit parameter
  double encoder_counts_per_output_shaft_turn_ = 360.0; // The encoder counts per output shaft turn will consider the encoder values as well as the gear ratio of your motor.
  double max_shaft_turn_per_minute_ = 170.0; // This is the maximum number of shaft turns per minute 



  //-----------------------------------------------
  // ROS messages 
  geometry_msgs::msg::Quaternion odom_quat;
  tf2::Quaternion quat;

  double ticks_per_meter = 1146.131;    // conversion factor from meter to ticks 
                                 // EMG30 has Encoder counts per output shaft turn 360 
                                 // Wheel is 100mm diameter.
                                 // circumference is then 2 * PI * Radius ==> 314,1 mm per turn
                                 // 1000 mm / 314,1 mm * 360 ticks per round
  double maximumSpeed = 0.9423;	 // This is the maximum speed for the EMG30 motor which is 
                                 // is 170 Rounds Per Minute. 
                                 // This makes 3 Rounds per second maximum speed
                                 // Which is 0,9423 m/s


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
  std::unique_ptr<I2cBus> i2c_bus;
  std::unique_ptr<MotorController> motor;

//---------------------------------------
 MD25MotorDriverROSWrapper(const rclcpp::NodeOptions & options) : Node("bus_master", options) {

    this->declare_parameter<double>("publish_current_speed_frequency", 10.0);
    this->declare_parameter<double>("publish_motor_status_frequency", 10.0);
    this->declare_parameter<double>("publish_odom_frequency", 10.0);
    this->declare_parameter<bool>("enable_odom", false);    
    this->declare_parameter<bool>("enable_status", false);       
    this->declare_parameter<bool>("enable_twist", enable_twist_default);    

    this->declare_parameter<double>("encoder_counts_per_output_shaft_turn", 360.0);
    this->declare_parameter<double>("max_shaft_turn_per_minute", 170.0);
    this->declare_parameter<double>("wheel_diameter", 0.1);
    this->declare_parameter<double>("width_robot", 0.1);
    this->declare_parameter<double>("distance_front_wheel", 0.26);
    this->declare_parameter<double>("distance_rear_wheel", 0.2);
   
    this->declare_parameter<double>("max_speed", 100.0);
   
    setParams();

    calculateInternalParams();

    //------------------------------------------
    i2c_bus.reset (new I2cBus ("/dev/i2c-1"));
    motor.reset(new hiwonder_driver());
    bool setup = motor->setup(this->get_logger (), i2c_bus);
    
    if(!setup){
      RCLCPP_ERROR(this->get_logger(), "failed to setup motor driver!");
    }

    bool resetAll = motor->resetEncoders (this->get_logger(), i2c_bus);
    if(!resetAll){
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
    
    // getting the motor unit parameter
    if (!this->get_parameter("encoder_counts_per_output_shaft_turn", encoder_counts_per_output_shaft_turn_)) {
      encoder_counts_per_output_shaft_turn_ = 360.0;
    }
    if (!this->get_parameter("max_shaft_turn_per_minute", max_shaft_turn_per_minute_)) {
      max_shaft_turn_per_minute_ = 170.0;
    }

    // getting the robot parameter values
    if (!this->get_parameter("wheel_diameter", wheel_diameter)) {
      wheel_diameter = 0.1;
    }
    if (!this->get_parameter("width_robot", width_robot)) {
      wheel_diameter = 0.1;
    }
    if (!this->get_parameter("distance_front_wheel", distance_front_wheel)) {
      wheel_diameter = 0.1;
    }
    if (!this->get_parameter("distance_rear_wheel", distance_rear_wheel)) {
      wheel_diameter = 0.1;
    }

    if (!this->get_parameter("max_speed", max_speed_)) {
      max_speed_ = 100.0;
    }



    if(!this->get_parameter("debug_mode",debug_mode_)){
      debug_mode_ = false;
    }

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

    // if(!ros::param::get("~enable_pid",enable_pid_)){
    //   enable_pid_ = false;
    // }

    RCLCPP_INFO(this->get_logger(),"bus master Parameters Set");
}

//---------------------------------------
/* calculate some internal values based on the input params */
void calculateInternalParams ()
{
    double wheelCircum =  PI * wheel_diameter; //0.65973
    ticks_per_meter =  1000.0 / wheelCircum * encoder_counts_per_output_shaft_turn_;    // conversion factor from meter to ticks 
                                 // EMG30 has Encoder counts per output shaft turn 360 
                                 // Wheel is 100mm diameter.
                                 // circumference is then 2 * PI * Radius ==> 314,1 mm per turn
                                 // 1000 mm / 314,1 mm * 360 ticks per round
    maximumSpeed = max_shaft_turn_per_minute_ * wheelCircum / 60.0; // This is the maximum speed based on the motor capabilities. 
    RCLCPP_INFO(this->get_logger(),"Maximum Motor Speed is %f ", maximumSpeed);
}

//---------------------------------------
/* Handle reset_encoders service message */
void callbackReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
{
    (void) request;
    motor->resetEncoders(this->get_logger (), i2c_bus);   
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
  std_msgs::msg::ByteMultiArray barry;
  bool success = true;
  std::vector<uint8_t> speeds = motor->getMotorsSpeed (this->get_logger(), i2c_bus, success);
  barry.data.clear();
  for (size_t i = 0; i < speeds.size(); ++i) {
    barry.data.push_back(i);
  }
  if (success)
  {
    current_speed_publisher_->publish(barry);
  }
  else {
        RCLCPP_WARN(this->get_logger(),"publishCurrentSpeed: Can not read motor speed");
  }

 }

//---------------------------------------
/* optional publish motor currents and battery voltage */
void publishMotorStatus(){
  bool success = true;
  std::vector<int> values = motor->readEncoders(this->get_logger(), i2c_bus, success);
  std_msgs::msg::ByteMultiArray barry;
  barry.data.clear();
  for (size_t i = 0; i < values.size(); ++i) {
    barry.data.push_back(i);
  }
  
  if (success)
  {
    motor_status_publisher_->publish(barry);
  }
  else {
        RCLCPP_WARN(this->get_logger(),"publishMotorStatus: Can not read motor encoder");
  }
 }

//---------------------------------------
/* Direct Stop all  Motors*/
void stop(){
    motor->stopMotors(this->get_logger (), i2c_bus);
 }

//---------------------------------------
/* Shutdown routine */
void shutdown(){
   stop();
 }

void read_encoder() {                  // Function to read and display value of encoder as a long
  bool success = true;

//  if (debug_mode_) {
//       RCLCPP_INFO(this->get_logger(),"read_encoder: Start Reading encoder values");
//  }
  std::vector<int> encoderValues = motor->readEncoders (this->get_logger(), i2c_bus, success);
  if (encoderValues.size() != 4)
  {
       RCLCPP_ERROR(this->get_logger(),"read_encoder: Can not read 4 encoder values");
       return;
  }
  front_left_encoder	= (double) encoderValues[0];			// Last encoder value front left
  front_right_encoder = (double) encoderValues[1];			// Last encoder value front right 
  rear_left_encoder	  = (double) encoderValues[2];			// Last encoder value rear left
  rear_right_encoder	= (double) encoderValues[3];			// Last encoder value rear right 
  if (debug_mode_) {
       RCLCPP_INFO(this->get_logger(),"read_encoder: Ended Reading encoder values");
  }
}

void publishOdom() {


  double dxy = 0.0;
  double dth = 0.0;
  rclcpp::Time current_time = this->get_clock()->now();
  double dt;
  double distance_left = 0.0;		 // calculated travel distance right 
  double distance_right = 0.0;	 // calculated travel distance left
  geometry_msgs::msg::TransformStamped t;

  read_encoder ();

  dt =  (current_time.seconds() - last_time.seconds());
  last_time = current_time;

  // calculate odometry
  distance_left = (front_left_encoder - front_left_encoder_old) / ticks_per_meter;
  distance_right = (front_right_encoder - front_right_encoder_old) / ticks_per_meter;

  front_left_encoder_old = front_left_encoder;
  front_right_encoder_old = front_right_encoder;
  rear_left_encoder_old = rear_left_encoder;
  rear_right_encoder_old = rear_right_encoder;

  dxy = (distance_left + distance_right) / 2.0;
  dth = (distance_right - distance_left) / width_robot;

  if(dxy != 0){
    x += dxy * cosf(theta);
    y += dxy * sinf(theta);
  }	

  if(dth != 0){
    theta += dth;
  }

  double vel_xy = dxy / dt;
  double vel_th = dth / dt;


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
 // HACK TODO  transform_broadcaster_->sendTransform(t);

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
  odom.twist.twist.linear.x = vel_xy;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = vel_th;

  //  publish the message
  odom_publisher_->publish(odom);

  if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(),
                  "Odom: dL=%f, dR=%f - dt:%f (x=%f, y=%f - t:%f)",
                  distance_left, distance_right, dt, pose.x, pose.y, pose.theta);
  }
}
 
//--------------------------------------------------------
/* Incoming cmd_vel message to Motor commands */
void twistToMotors(const geometry_msgs::msg::Twist &msg){

  geometry_msgs::msg::Twist twist = msg;	
  double vel_x = twist.linear.x;
  double vel_y = twist.linear.y;
  double vel_th = twist.angular.z;
 

  velocity_front_left   =  vel_x - vel_y - vel_th * (distance_front_wheel + distance_rear_wheel);
  velocity_front_right  =  vel_x + vel_y + vel_th * (distance_front_wheel + distance_rear_wheel);
  velocity_rear_left    =  vel_x + vel_y - vel_th * (distance_front_wheel + distance_rear_wheel);
  velocity_rear_right   =  vel_x - vel_y + vel_th * (distance_front_wheel + distance_rear_wheel);

  int front_right = convertVelocityToMotorSpeed(velocity_front_right);
  int front_left = convertVelocityToMotorSpeed(velocity_front_left);
  int rear_right = convertVelocityToMotorSpeed(velocity_rear_right);
  int rear_left = convertVelocityToMotorSpeed(velocity_rear_left);

  if (debug_mode_) {
       RCLCPP_INFO(this->get_logger(),"twistToMotors: %d %d %d %d", front_right, front_left, rear_right, rear_left);
  }
  motor->setMotorsSpeed (this->get_logger(), i2c_bus, front_left, front_right, rear_left, rear_right);
}

// Convert from m/s speed into the command values from the 
// Motor and the number of the speed values in the motor driver
int convertVelocityToMotorSpeed (double speed)
{
    if ((speed < 0.0001) && (speed > -0.0001))
    {
        //This is nearly zero so stop
        return 0;
    }
    if (speed >= maximumSpeed)
    {
        return (int) max_speed_;
    }
    if (speed <= - maximumSpeed)
    {
        return -(int) max_speed_;
    }
    // Maximum speed is divided by the maximum speed to get the 
    // steps in which the speed can be controlled. 
    // Intended speed divided by these steps gives the number for the motor
    double emg30Speed = (speed / (maximumSpeed / max_speed_));
    if (emg30Speed  > max_speed_)
    {
        emg30Speed = max_speed_;
    }
    if (emg30Speed  < -max_speed_)
    {
        emg30Speed = -max_speed_;
    }
    return (int)emg30Speed ;
}

//---------------------------------------
bool resetAll(){
  bool reset  = motor->resetEncoders(this->get_logger (), i2c_bus);
  RCLCPP_INFO(this->get_logger(),"MD25 Reset ALL");
  return reset;
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

    RCLCPP_INFO(node->get_logger(),"Motor Driver v0.0.1 Started");
    return 0;
 }
