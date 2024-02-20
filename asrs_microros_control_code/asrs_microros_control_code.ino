#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>


// custom includes
#include <Servo.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float64MultiArray msg;
std_msgs__msg__Float64MultiArray msg_pub;
rclc_executor_t executor_sub;
rclc_executor_t executor_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer_pub;
bool micro_ros_init_successful;


enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Defining the variables
double angle_deg[10];
double angle_rad[10];
double goal_steps[5];
const double pi=3.1415;


// Number of steps per output rotation
const int stepsPerRevolution = 100;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// for base rotation- Link 0
#define dirPin0 39
#define stepPin0 41
const int stepsPerRevolution0=1600;

// Define stepper motor connections and steps per revolution:
// for link 1

#define dirPin1 6
#define stepPin1 7
const int stepsPerRevolution1=1600;
const int gear_ratio1=20;

// for link 2 

#define dirPin2 11
#define stepPin2 12
const int stepsPerRevolution2=3200;
const int gear_ratio2=5;


// for servo Pitch movement and Gripper

Servo servog; // Servo for gripper
float angle_g=0.0;
Servo servop; // Servo for pitch
float angle_p=0.0;

#define LED_PIN 13

void run_robot();

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(void * msgin)
{  

  std_msgs__msg__Float64MultiArray * msg = (std_msgs__msg__Float64MultiArray *)msgin;
  
  for (int i=0;i<10;i++)
  {
    angle_rad[i] = msg->data.data[i];
    angle_deg[i]=angle_rad[i]*180/pi;
  }
   
  goal_steps[0]=angle_deg[0]*stepsPerRevolution0/360;
  goal_steps[1]=angle_deg[1]*stepsPerRevolution1*gear_ratio1/360;
  goal_steps[2]=angle_deg[2]*stepsPerRevolution2*gear_ratio2/360;
  goal_steps[3]=angle_deg[3];
  goal_steps[4]=angle_deg[4];  

  msg_pub.data.data[0]=goal_steps[0];
  msg_pub.data.data[1]=goal_steps[1];
  msg_pub.data.data[2]=goal_steps[2];
  msg_pub.data.data[3]=goal_steps[3];
  msg_pub.data.data[4]=goal_steps[4];

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

void timer_callback_pub(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {       
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));    
  }
  RCLC_UNUSED(timer);
}

bool create_entities()
{
// Subscriber

  allocator = rcl_get_default_allocator();

// create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator)); 

// create node
  RCCHECK(rclc_node_init_default(&node, "asrs_control_node", "", &support));

// create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "processed_joint_states"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "goal_publisher"));

  // create timer for sub,
  const unsigned int timer_timeout = 2000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // // create timer for pub,
  const unsigned int timer_timeout_pub = 2000;
  RCCHECK(rclc_timer_init_default(
    &timer_pub,
    &support,
    RCL_MS_TO_NS(timer_timeout_pub),
    timer_callback_pub));

  //Initiate array for received message
  msg.data.capacity = 10; 
  msg.data.size = 10;
  msg.data.data = (double*) malloc(msg.data.capacity * sizeof(double));

  msg.layout.dim.capacity = 10;
  msg.layout.dim.size = 10;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(size_t i = 0; i < msg.layout.dim.capacity; i++){
      msg.layout.dim.data[i].label.capacity = 10;
      msg.layout.dim.data[i].label.size = 0;
      msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor- subscriber
  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 2, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor_sub, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // initiate array for published message
  msg_pub.data.capacity = 5; 
  msg_pub.data.size = 5;
  msg_pub.data.data = (double*) malloc(msg_pub.data.capacity * sizeof(double));

  msg_pub.layout.dim.capacity = 5;
  msg_pub.layout.dim.size = 5;
  msg_pub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(size_t i = 0; i < msg_pub.layout.dim.capacity; i++){
      msg_pub.layout.dim.data[i].label.capacity = 5;
      msg_pub.layout.dim.data[i].label.size = 0;
      msg_pub.layout.dim.data[i].label.data = (char*) malloc(msg_pub.layout.dim.data[i].label.capacity * sizeof(char));
  }

// create executor- publisher
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer_pub));

  return true;
}

void setup() 
{
  Serial.begin(9600);
  set_microros_transports();
// Link 0
  pinMode(stepPin0,OUTPUT);
  pinMode(dirPin0,OUTPUT);

// for link 1
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  
// for link 2
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

//servo pitch- Link 4
  servop.attach(4);

//servo gripper- Link 5
  servog.attach(3);

state = WAITING_AGENT;

}


void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  // rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer_pub);
  rclc_executor_fini(&executor_pub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


void run_robot(){

// Link 0

 // Set the spinning direction clockwise:
 if(goal_steps[0]<0)
 {
  digitalWrite(dirPin0, LOW);
 }else
 {
  digitalWrite(dirPin0,HIGH);
 }

// Spin the stepper motor 0 revolution slowly:
 for (int i = 0; i < abs(goal_steps[0]); i++) {
    // These four lines result in 1 step:
   digitalWrite(stepPin0, HIGH);
   delayMicroseconds(1000);
   digitalWrite(stepPin0, LOW);
   delayMicroseconds(1000);
 }

//  delay(1000);

// Link 1

 // Set the spinning direction clockwise:
 if(goal_steps[1]<0)
 {
  digitalWrite(dirPin1, LOW);
 }else
 {
  digitalWrite(dirPin1,HIGH);
 }

// Spin the stepper motor 1 revolution slowly:
 for (int i = 0; i < abs(goal_steps[1]); i++) {
    // These four lines result in 1 step:
   digitalWrite(stepPin1, HIGH);
   delayMicroseconds(1000);
   digitalWrite(stepPin1, LOW);
   delayMicroseconds(1000);
 }
//  delay(1000);

// Link 2

 // Set the spinning direction clockwise:
 if(goal_steps[2]<0)
 {
  digitalWrite(dirPin2, LOW);
 }else
 {
  digitalWrite(dirPin2,HIGH);
 }

// Spin the stepper motor 2 revolution slowly:
 for (int i = 0; i < abs(goal_steps[2]); i++) {
    // These four lines result in 1 step:
   digitalWrite(stepPin2, HIGH);
   delayMicroseconds(1000);
   digitalWrite(stepPin2, LOW);
   delayMicroseconds(1000);
 }

//  delay(1000);


//  Link 3- Servo pitch
 angle_p=angle_p+goal_steps[3];
 servop.write(angle_p);
 delay(1000);

// Link 4- Servo gripper
 angle_g=angle_g+goal_steps[4];
 servog.write(angle_g);
 delay(1000);

} 


void loop() 
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_sub, RCL_S_TO_NS(1));
        delay(1000);
        run_robot();
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
        delay(1000);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
