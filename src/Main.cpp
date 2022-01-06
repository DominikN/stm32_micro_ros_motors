#include <Arduino.h>
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "EncMotors.h"

#define CLIENT_IP "192.168.1.177"
#define AGENT_IP "192.168.1.176"
#define AGENT_PORT 8888
#define NODE_NAME "stm32_node"

#define LED_PIN PE3

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("o");          \
    }                              \
  }

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

IPAddress client_ip;
IPAddress agent_ip;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};

void error_loop() {
  while (1) {
    Serial.printf("in error loop");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  Serial.printf("[%s]: I heard: [%s]\r\n", NODE_NAME,
                micro_ros_string_utilities_get_c_str(msg->data));
}

char buffer[100];

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    static int cnt = 0;
    sprintf(buffer, "Hello World: %d, sys_clk: %d", cnt++, xTaskGetTickCount());
    Serial.printf("Publishing: %s\r\n", buffer);

    msg.data = micro_ros_string_utilities_set(msg.data, buffer);

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

static void rclc_spin_task(void *p);
static void chatter_publisher_task(void *p);
static void runtime_stats_task(void *p);

EncMotors Motor1;

void setup() {
  portBASE_TYPE s1, s2, s3;

  // Open serial communications and wait for port to open:
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);

  // ================== TESTING MOTORS ==========================
  // int i = 100;

  // Motor1.setPower(35);

  // while (1) {
  //   // Serial.printf("PWM = %d\r\n", i);
  //   // i--;
  //   // Motor1.setPower(i);
  //   // Motor1.setPower(-30);
  //   // delay(1000);
  //   // Motor1.setPower(30);
  //   // delay(1000);
  // }

  // =============================================================

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(AGENT_IP);

  Serial.printf("Connecting to agent: ");
  Serial.println(agent_ip);

  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              AGENT_PORT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // create spin task
  s1 = xTaskCreate(rclc_spin_task, "rclc_spin_task",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 0,
                   NULL);

  // create spin task
  s2 = xTaskCreate(runtime_stats_task, "runtime_stats_task",
                   configMINIMAL_STACK_SIZE + 2000, NULL, tskIDLE_PRIORITY + 1,
                   NULL);

  // check for creation errors
  if (s1 != pdPASS || s2 != pdPASS) {
    Serial.println(F("Creation problem"));
    while (1)
      ;
  }

  // start FreeRTOS
  vTaskStartScheduler();
}

static void rclc_spin_task(void *p) {
  UNUSED(p);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    RCSOFTCHECK(rclc_executor_spin(&executor));
    vTaskDelayUntil(&xLastWakeTime, 10);
  }
}

static void runtime_stats_task(void *p) {
  char buf[2000];
  Serial.printf("runtime stats task started");
  while (1) {
    vTaskGetRunTimeStats(buf);
    Serial.printf("\r\n%s\r\n-------------", buf);
    vTaskDelay(2000);
  }
}

void loop() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(1000);
}

// =========== Runtime stats ====================

HardwareTimer stats_tim(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void) {
  //   m1_pwm.setPWM(1, M1_PWM, 1000, power);
  stats_tim.setPrescaleFactor(
      1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
              // Hz (from prediv'd by 1 clocksource of 168 MHz)
  stats_tim.setOverflow(0xffffffff);  // Set overflow to 32761 => timer
                                      // frequency = 65522 Hz / 32761 = 2 Hz
  stats_tim.refresh();                // Make register changes take effect
  stats_tim.resume();                 // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) { return stats_tim.getCount(); }