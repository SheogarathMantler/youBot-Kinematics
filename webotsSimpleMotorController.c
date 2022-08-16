/*
 * File:          my_youbot_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <arm.h>
#include <base.h>
#include <gripper.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  WbDeviceTag motor1 = wb_robot_get_device("arm1");
  WbDeviceTag motor2 = wb_robot_get_device("arm2");
  WbDeviceTag motor3 = wb_robot_get_device("arm3");
  WbDeviceTag motor4 = wb_robot_get_device("arm4");
  WbDeviceTag motor5 = wb_robot_get_device("arm5");
  passive_wait(2.0);
  wb_motor_set_position(motor1, 0);
  wb_motor_set_position(motor2, 0);
  wb_motor_set_position(motor3, 0);
  wb_motor_set_position(motor4, 1.57);
  wb_motor_set_position(motor5, 0);
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
