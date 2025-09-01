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
#include <std_msgs/msg/int16_multi_array.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/

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
rcl_publisher_t int16array_pub;
rcl_subscription_t int16array_sub;

/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__Int16MultiArray int16array_send_msg;
std_msgs__msg__Int16MultiArray int16array_recv_msg;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/

void int16array_callback(const void *msgin) {
  std_msgs__msg__Int16MultiArray *int16array_recv_msg = (std_msgs__msg__Int16MultiArray *)msgin;
  /*
   * Do something with your receive message
   */
  for (int i = 0; i < 18; i++) {
    int16array_send_msg.data.data[i] = int16array_recv_msg->data.data[i];
  }
  rcl_publish(&int16array_pub, &int16array_send_msg, NULL);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    /*
       TODO : Add any periodic functionality inside here if needed
    */
  }
}

/*
   Create object (Initialization)
*/
bool create_entities() {
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  const char *node_name = "esp_node";
  const char *ns = "";
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
      &int16array_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "/esp/int16array_pub", &rmw_qos_profile_default);

  rclc_subscription_init(
      &int16array_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "/esp/int16array_sub", &rmw_qos_profile_default);

  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 2;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

/*
 * Clean up all the created objects
 */
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  /*
   * TODO : Make sure the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&int16array_pub, &node);
  rcl_subscription_fini(&int16array_sub, &node);
}

void setup() {
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888");

  /*
   * Optional, setup output pin for LEDs
   */

  /*
   * TODO : Initialize the message data variable
   */
  static int16_t array_data[18] = {0}; // Initialize array with 18 zeros
  int16array_send_msg.data.capacity = 18; // number of array length
  int16array_send_msg.data.size = 18;     // number of array length
  int16array_send_msg.data.data = array_data;

  int16array_recv_msg.data.capacity = 18; // number of array length
  int16array_recv_msg.data.size = 18;     // number of array length
  int16array_recv_msg.data.data = array_data;

  /*
   * Setup first state
   */
  state = WAITING_AGENT;
}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
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
   * TODO : Add additional functionality here
   */
}
