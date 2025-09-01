/*
 * TODO : Include your necessary header here
*/


/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/
rcl_publisher_t float32array_pub;
rcl_subscription_t float32array_sub;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__Float32MultiArray float32array_send_msg;
std_msgs__msg__Float32MultiArray float32array_recv_msg;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/

void float32array_callback(const void *msgin) {

  std_msgs__msg__Float32MultiArray * float32array_recv_msg = (std_msgs__msg__Float32MultiArray *)msgin;
  /*
   * Do something with your receive message
   */

  for(int i=0 ; i<18 ; i++){
    float32array_send_msg.data.data[i] = float32array_recv_msg->data.data[i];
  }

  // Serial.print(" data:");
  // Serial.print(float32array_send_msg.data.data[0]);
  // Serial.print(" ");
  // Serial.println(float32array_send_msg.data.data[1]);

  rcl_publish(&float32array_pub, &float32array_send_msg, NULL);

}

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   (void) last_call_time;
//   if (timer != NULL) {

//     /*
//        TODO : Publish anything inside here
       
//        For example, we are going to echo back
//        the float32array_sub data to float32array_pub data,
//        so we could see the data reflect each other.
//        And also keep incrementing the int16_pub
//     */

//     rcl_publish(&float32array_pub, &float32array_send_msg, NULL);
//     float32array_send_msg.data.data[0]++;
//     float32array_send_msg.data.data[1]++;
//     float32array_send_msg.data.data[1]++;
//   }
// }

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  const char * node_name = "servo_driver_node";
  const char * ns = "";
  const int domain_id = 0;
  
  /*
   * Initialize node
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  
  /*
   * TODO : Init your publisher and subscriber 
   */
  rclc_publisher_init(
    &float32array_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/esp/float32array_pub", &rmw_qos_profile_default);

  rclc_subscription_init(
    &float32array_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/esp/float32array_sub", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  // const unsigned int timer_timeout = 10;
  // rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &float32array_sub, &float32array_recv_msg, &float32array_callback, ON_NEW_DATA);
  // rclc_executor_add_timer(&executor, &timer);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&float32array_pub, &node);
  rcl_subscription_fini(&float32array_sub, &node);  
}

void setup() {
  Serial.begin(115200);
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /*
   * Optional, setup output pin for LEDs
   */

  /*
   * TODO : Initialze the message data variable
   */
  float array_data[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
  float32array_send_msg.data.capacity = 18; // number of array length
  float32array_send_msg.data.size = 18;     // number of array length
  float32array_send_msg.data.data = array_data;

  float32array_recv_msg.data.capacity = 18; // number of array length
  float32array_recv_msg.data.size = 18;     // number of array length
  float32array_recv_msg.data.data = array_data;
  /*
   * Setup first state
   */
  state = WAITING_AGENT;

}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
   * from the example
   * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
   * 
   */
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  /*
   * Output LED when in AGENT_CONNECTED state
   */

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}