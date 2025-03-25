#include <micro_ros_arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ----------------------------
// Encoder & Motor Definitions
// ----------------------------
#define M1_CH_A  18
#define M1_CH_B  19
#define M2_CH_A  23
#define M2_CH_B  25

#define PPR_M1   722*4
#define PPR_M2   722*4
#define GEAR_RATIO (34.0f / 20.0f)
#define WHEEL_DIAMETER_CM 18.0
const float wheelCircumference_m = (WHEEL_DIAMETER_CM * 3.14159265) / 100.0;

#define UPDATE_INTERVAL 1000 
#define LED_PIN 13

// Encoder counts
volatile long m1Count = 0;
volatile long m2Count = 0;
long m1LastCount = 0;
long m2LastCount = 0;

// ----------------------------
// IMU Definitions
// ----------------------------
#define DEG2RAD 0.01745329252
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SemaphoreHandle_t imu_mutex = NULL;

// ----------------------------
// ROS Variables
// ----------------------------
rcl_publisher_t motor_publisher;
rcl_publisher_t imu_publisher;
std_msgs__msg__String motor_msg;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t motor_timer;
rcl_timer_t imu_timer;
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ----------------------
// Encoder ISRs
// ----------------------
void IRAM_ATTR m1ISR() {
  if (digitalRead(M1_CH_B) == LOW) {
    m1Count++;
  } else {
    m1Count--;
  }
}

void IRAM_ATTR m2ISR() {
  if (digitalRead(M2_CH_B)) m2Count--;
  else m2Count++;
}

// ----------------------
// Motor Timer Callback
// ----------------------
void motor_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    long diffM1 = m1Count - m1LastCount;
    m1LastCount = m1Count;
    long diffM2 = m2Count - m2LastCount;
    m2LastCount = m2Count;

    float rawRPM_M1 = (diffM1 / (float)PPR_M1) * 60.0;
    float rawRPM_M2 = (diffM2 / (float)PPR_M2) * 60.0;
    float outputRPM_M1 = rawRPM_M1 * GEAR_RATIO;
    float outputRPM_M2 = rawRPM_M2 * GEAR_RATIO;

    char dirM1 = (rawRPM_M1 >= 0) ? 'F' : 'R';
    char dirM2 = (rawRPM_M2 >= 0) ? 'F' : 'R';

    float velM1 = (fabs(outputRPM_M1) / 60.0) * wheelCircumference_m;
    float velM2 = (fabs(outputRPM_M2) / 60.0) * wheelCircumference_m;

    char buffer[128];
    snprintf(buffer, sizeof(buffer),
      "[%lu ms] M1: %.2f m/s, %.2f RPM, %c | M2: %.2f m/s, %.2f RPM, %c",
      millis(), velM1, outputRPM_M1, dirM1, velM2, outputRPM_M2, dirM2
    );

    motor_msg.data.data = buffer;
    motor_msg.data.size = strlen(buffer);
    motor_msg.data.capacity = sizeof(buffer);
    RCSOFTCHECK(rcl_publish(&motor_publisher, &motor_msg, NULL));
  }
}

// ----------------------
// IMU Task & Callback
// ----------------------
void sensorReadTask(void * parameter) {
  (void) parameter;
  while (1) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    float roll = euler.x() * DEG2RAD;
    float pitch = euler.y() * DEG2RAD;
    float yaw = euler.z() * DEG2RAD;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
      imu_msg.header.frame_id.data = "imu_link";
      imu_msg.header.frame_id.size = strlen("imu_link");
      imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
      imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
      imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
      imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
      imu_msg.angular_velocity.x = gyro.x();
      imu_msg.angular_velocity.y = gyro.y();
      imu_msg.angular_velocity.z = gyro.z();
      imu_msg.linear_acceleration.x = linAccel.x();
      imu_msg.linear_acceleration.y = linAccel.y();
      imu_msg.linear_acceleration.z = linAccel.z();
      xSemaphoreGive(imu_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && xSemaphoreTake(imu_mutex, portMAX_DELAY)) {
    uint32_t micros_val = micros();
    imu_msg.header.stamp.sec = micros_val / 1000000UL;
    imu_msg.header.stamp.nanosec = (micros_val % 1000000UL) * 1000UL;
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    xSemaphoreGive(imu_mutex);
  }
}

// ----------------------
// Setup & Loop
// ----------------------
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  set_microros_wifi_transports("robot_1", "robot123", "192.168.0.129", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  delay(2000);

  // Encoder setup
  pinMode(M1_CH_A, INPUT_PULLUP);
  pinMode(M1_CH_B, INPUT_PULLUP);
  pinMode(M2_CH_A, INPUT_PULLUP);
  pinMode(M2_CH_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M1_CH_A), m1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(M2_CH_A), m2ISR, FALLING);

  // IMU setup
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    error_loop();
  }
  bno.setExtCrystalUse(true);
  imu_mutex = xSemaphoreCreateMutex();
  if (!imu_mutex) error_loop();

  // ROS setup
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

  // Motor publisher
  RCCHECK(rclc_publisher_init_default(
    &motor_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motors_info"
  ));

  // IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"
  ));

  // Timers
  RCCHECK(rclc_timer_init_default(
    &motor_timer,
    &support,
    RCL_MS_TO_NS(UPDATE_INTERVAL),
    motor_timer_callback
  ));
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(10),
    imu_timer_callback
  ));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &motor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

  // IMU task
  xTaskCreate(sensorReadTask, "IMU_Task", 4096, NULL, 1, NULL);

  Serial.println("Combined Motor & IMU Node Initialized");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(5);
}
