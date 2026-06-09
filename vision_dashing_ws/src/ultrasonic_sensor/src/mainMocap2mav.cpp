#include <iostream>
#include "UltrasonicSensor.h"
#include <queue>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ultrasonic_sensor/msg/ultrasonic_msg.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "MedianFilter.h"
#include <sys/time.h>

#define trigger 23  // GPIO pin: ultrasonic trigger
#define echo    22  // GPIO pin: ultrasonic echo
#define timeout 150 // echo timeout in milliseconds

bool init = true;

typedef std::queue<double> queue_t;
queue_t stack_z;

double integral_z      = 0;
double previous_distance = 0;
int    window          = 6;

struct timeval start;
struct timeval end;
double dt;
int RATE = 30;

double Pred_P_k_1 = 0.003; // initial Kalman covariance estimate
double Xk;
double Pk;
double Q = 0.5e-3;   // process noise covariance
double R = 2.92e-3;  // measurement noise covariance

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("Basic_kalman_filter_for_ultra_Sensor");

  auto Sonar1 = std::make_shared<UltrasonicSensor>();
  Sonar1->setParameters(trigger, echo, timeout);

  auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  auto reliable_qos    = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  auto ultrasonic_sensor_sub = node->create_subscription<sensor_msgs::msg::Range>(
    "/ultrasonic/sonar", best_effort_qos,
    std::bind(&UltrasonicSensor::setSensorData, Sonar1, std::placeholders::_1));

  // Replaces: handler.publish("UltrasonicSensor/platform", &temp_plat_filtr)
  auto ultrasonic_pub = node->create_publisher<ultrasonic_sensor::msg::UltrasonicMsg>(
    "UltrasonicSensor/platform", reliable_qos);

  rclcpp::WallRate loop_rate(30);

  auto state       = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();
  auto state_filtr = std::make_shared<ultrasonic_sensor::msg::UltrasonicMsg>();

  state->z_position = Sonar1->getDistance();

  double Pred_Xk_1 = state->z_position;

  MedianFilter<double, 5> f;

  dt = 1.0 / RATE;

  while (rclcpp::ok())
  {
    gettimeofday(&start, NULL);

    KF(Pred_Xk_1, Pred_P_k_1, state->z_position, &Xk, &Pk, Q, R);

    Pred_P_k_1 = Pk;
    Pred_Xk_1  = Xk;

    state->z_position        = Sonar1->getDistance();
    state_filtr->z_position  = Xk;
    state_filtr->is_valid    = Sonar1->GetValidity();  // replaces temp_plat_filtr.isValid
	
    f.addSample(EstimatedVelocity(
      &state_filtr->z_position, &previous_distance,
      &stack_z, &init, dt, &integral_z, window));

    state_filtr->z_velocity = f.isReady() ? f.getMedian() : 0.0;
	// state_filtr->is_valid = true;
    // Replaces: handler.publish("UltrasonicSensor/platform", &temp_plat_filtr)
    ultrasonic_pub->publish(*state_filtr);

    rclcpp::spin_some(node);
    loop_rate.sleep();

    gettimeofday(&end, NULL);
    dt = ((end.tv_sec  - start.tv_sec)  * 1000000.0
        + (end.tv_usec - start.tv_usec)) * 1.0e-6;
  }
}