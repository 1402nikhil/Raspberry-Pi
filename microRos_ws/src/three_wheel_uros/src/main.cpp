#include <Arduino.h>
#include <Encoder.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define LPWM_1 5
#define RPWM_1 4
#define LPWM_2 2
#define RPWM_2 3


Encoder enc1(6,7);
Encoder enc2(8,9);

rcl_publisher_t encoder_readings_pub_;
rcl_publisher_t check_pub_;

rcl_subscription_t wheel_vel_sub_;
rcl_subscription_t pid_values_sub_;

std_msgs__msg__Float32MultiArray encoder_readings_;
std_msgs__msg__Bool check_;
std_msgs__msg__Float32MultiArray wheel_vel_;
std_msgs__msg__Float32MultiArray pid_values_;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t mc_node;
rcl_timer_t encoder_readings_timer;
rcl_timer_t check_timer;

float v1 = 0, v2 = 0, v3 = 0;
float pid_values[4] = {0};


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void enc_read_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&encoder_readings_pub_, &encoder_readings_, NULL));
  }
}

void check_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&check_pub_, &check_, NULL));
  }
}

void wheel_vel_callback(const void * msgin)
{  
  if(msgin == NULL){
    return;
  }
  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  if((msg->data.data == NULL) || (msg->data.size < 3)){
    return;
  }

  v1 = msg->data.data[0];
  v2 = msg->data.data[1];
  v3 = msg->data.data[2];
}

void pid_callback(const void * msgin)
{  
  if(msgin == NULL){
    return;
  }

  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  if((msg->data.data == NULL) || (msg->data.size < 4)){
    return;
  }

  pid_values[0] = msg->data.data[0];
  pid_values[1] = msg->data.data[1];
  pid_values[2] = msg->data.data[2];
  pid_values[3] = msg->data.data[3];
}

void encoderReadings(){
  encoder_readings_.data.data[0] = enc1.read();
  encoder_readings_.data.data[1] = enc2.read();
}

void moveMotors(){
    
  // if(v1 > 50){
  //   v1 = 50;
  // }
  // if(v2 > 50){
  //   v2 = 50;
  // }

  if(v1 > 0){
    analogWrite(LPWM_1, v1);
    analogWrite(RPWM_1, 0);
  }
  else{
    analogWrite(LPWM_1, 0);
    analogWrite(RPWM_1, abs(v1));
  }

  if(v2 > 0){
    analogWrite(LPWM_2, v2);
    analogWrite(RPWM_2, 0);
  }
  else{
    analogWrite(LPWM_2, 0);
    analogWrite(RPWM_2, abs(v2));
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  pinMode(LPWM_1, OUTPUT);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&mc_node, "micro_ros_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_readings_pub_,
    &mc_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_encoders"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &check_pub_,
    &mc_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "communication_check"));
      
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &wheel_vel_sub_,
    &mc_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "cmd_vel"));
      
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &pid_values_sub_,
    &mc_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "set_pid"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &encoder_readings_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    enc_read_callback));

  // create timer,
  const unsigned int timer_timeout1 = 1000;
  RCCHECK(rclc_timer_init_default(
    &check_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout1),
    check_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &encoder_readings_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &check_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &wheel_vel_sub_, &wheel_vel_, &wheel_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_values_sub_, &pid_values_, &pid_callback, ON_NEW_DATA));

  wheel_vel_.data.size = 3;
  wheel_vel_.data.capacity = 3;
  wheel_vel_.data.data = (float*) malloc(3 * sizeof(float));

  wheel_vel_.data.data[0] = 0;
  wheel_vel_.data.data[1] = 0;
  wheel_vel_.data.data[2] = 0;

  // encoder_readings_ = { .data = { .data = NULL, .size = 0, .capacity = 0}};
  encoder_readings_.data.size = 2;
  encoder_readings_.data.capacity = 2;
  encoder_readings_.data.data = (float*) malloc(2 * sizeof(float));

  encoder_readings_.data.data[0] = 0;
  encoder_readings_.data.data[1] = 0;

  check_.data = true;
  // pid_values_ = { .data = { .data = NULL, .size = 0, .capacity = 0}};
  pid_values_.data.size = 4;
  pid_values_.data.capacity = 4;
  pid_values_.data.data = (float*) malloc(4 * sizeof(float));

  pid_values_.data.data[0] = 0;
  pid_values_.data.data[1] = 0;
  pid_values_.data.data[2] = 0;
  pid_values_.data.data[3] = 0;

}

void loop() {
  // delay(100);
  encoderReadings();
  moveMotors();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}