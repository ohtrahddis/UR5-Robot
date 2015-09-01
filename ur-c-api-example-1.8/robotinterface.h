#ifndef _ROBOTINTERFACE_H
#define _ROBOTINTERFACE_H

#include <inttypes.h>

/*!
 * \file robotinterface.h
 *
 * \section UR_COMM_API Universal Robots API, interface to the real thing.
 *
 *
 * How to use it (meta code);
 * \code
 * double[] positions[6];
 * robotinterface_open();
 * while(!endcondition) { // At ROBOT_CONTROLLER_FREQUENCY times per second
 *   robotinterface_read_robot_state_blocking();
 *   robotinterface_get_actual_positions(&positions);
 *     >>> various calculations <<<
 *   robotinterface_command_position_velocity_acceleration( xxx, yyy, zzz);
 *   robotinterface_send_robot_command();
 * }
 * robotinterface_close();
 * \endcode
 *
 * \subsection feature Feature:
 *
 * By setting #robotinterface_enable_real_robot() to \c false,
 * you can test all your code without actually running on a robot.
 * Collision detection, joint limit violation and so on is still active when the real robot is disabled.
 *
 *
 * \subsection expect What can you expect:
 *
 *   If the #robotinterface_is_real_robot_running() function returns \c true, you can expect the
 * real robot to return its joint positions and speeds.
 * - robotinterface_get_actual_positions(&doublearray); returns the internally measured position of the robot
 * - robotinterface_get_target_positions(&doublearray); returns where you told the robot to be.
 * and so forth with speed, acceleration, and current.
 *
 *
 * \subsection what What does it do:
 *
 * - Checks for collisions and joint violations.
 * - Sends commands to the robot
 * - Keeps track of time and synchronizes your software with the robot
 * - Provides a security stop feature
 */

/* robot modes */
#define ROBOT_RUNNING_MODE 0
#define ROBOT_FREEDRIVE_MODE 1
#define ROBOT_READY_MODE 2
#define ROBOT_INITIALIZING_MODE 3
#define ROBOT_SECURITY_STOPPED_MODE 4
#define ROBOT_EMERGENCY_STOPPED_MODE 5
#define ROBOT_FATAL_ERROR_MODE 6
#define ROBOT_NO_POWER_MODE 7
#define ROBOT_NOT_CONNECTED_MODE 8
#define ROBOT_SHUTDOWN_MODE 9

#ifdef __cplusplus
extern "C" {
#endif

  uint64_t robotinterface_get_step();
  double robotinterface_get_time();
  double robotinterface_get_time_per_step();

  /**
   * \name Robot commands
   * This is where the robot command methods start.
   * @{
   */
  void robotinterface_command_position_velocity_acceleration(const double *q,
                                                             const double *qd,
                                                             const double
                                                       *qdd);

  /**
   * \name Robot commands
   * This is where the robot command methods start.
   * @{
   */
  void robotinterface_command_velocity_acceleration(const double *q,
                                                             const double *qd,
                                                             const double
                                                             *qdd);
  void robotinterface_command_velocity(const double *qd);
  void robotinterface_command_joint_position_velocity_acceleration(int joint,
                                                                   double q,
                                                                   double qd,
                                                                   double
                                                                   qdd);

  void robotinterface_command_velocity_security_torque_control_torque(const double *qd,
                                                             const double *security_torque,
                                                             const double *control_torque,
                                                             const double *softness
                                                             );

  void robotinterface_command_torque(const double *qd,
                                     const double *security_torque,
                                     const double *control_torque
                                     );

  void robotinterface_command_empty_command();

  void robotinterface_master_goto_bootloader_mode();

  /* @{ */
  /** Value must be 0 or 1, port can be 0..9 */
  void robotinterface_command_digital_out_port(int port, int value);
  void robotinterface_command_digital_out_bits(unsigned short bits);
  /** Value must be 0 or 1, port can be 0..9 */
  void robotinterface_command_analog_out_port(int port, double value);

  /**
   * Value must be 0 or 1, port can be 0..9
   * For port 0 and 1, the controller box analog inputs;
   * - Range must be 0,1,2,3
   * For port 2 and 3, the tool board analog inputs;
   * - Range 0 is 0..5 Volt
   * - Range 1 is 0..10 Volts
   * - Range 4 is 4..20 mA
   */
  void robotinterface_command_analog_input_range_port(int port, int range);
  void robotinterface_command_analog_output_domain(int port, int type);
  void robotinterface_command_tool_output_voltage(unsigned char value);

  /* The TCP is regarded as part of the robot
   * Get and set methods for TCP and TCP payload are as follows
   */
  void robotinterface_set_tcp(const double *tcp_pose);
  void robotinterface_set_tcp_payload_cog(const double *tcp_payload_cog);
  void robotinterface_set_tcp_payload(double tcp_payload);
  void robotinterface_get_tcp(double *tcp_pose);
  void robotinterface_get_tcp_payload_cog(double *tcp_pose);
  double robotinterface_get_tcp_payload();
  /* @} */
  /* @} */

  /* Funtions to set/get force and torque (wrench) at the TCP */
  void robotinterface_set_tcp_wrench(const double *new_tcp_wrench, const int in_base_coord);
  void robotinterface_get_tcp_wrench(double *gotten_tcp_wrench);
  int robotinterface_is_tcp_wrench_in_base_coord();


  /**
   * \name Communication
   * This is where the robot communcation methods start.
   *
   * \{
   * \brief Initialize connection to the robot.
   *
   * If robot is directly connected, it returns TRUE,
   * otherwise it will return false and enter simulation mode.
   *
   * simulation mode can also be simulated by setting argument
   * simulated = 1.
   */
  int robotinterface_open(int open_simulated);
  int robotinterface_do_open(int open_simulated);
  int robotinterface_open_allow_bootloader(int open_simulated);
  void robotinterface_read_state_blocking();
  void robotinterface_send();

  /* ! physical robot connected? (Ethernet talking to coldfire) */
  int robotinterface_is_robot_connected();


  /* ! Shuts down the communication channel to the robot. */
  int robotinterface_close();


  /**
   * Returns robot mode, which is one of the following:
   *
   * - ROBOT_RUNNING_MODE
   * - ROBOT_READY_MODE
   * - ROBOT_INITIALIZING_MODE
   * - ROBOT_STOPPED_MODE
   * - ROBOT_SECURITY_STOPPED_MODE
   * - ROBOT_FATAL_ERROR_MODE
   * - ROBOT_NOT_CONNECTED_MODE
   */
  uint8_t robotinterface_get_robot_mode();

  /**
   * Sets the robot in ROBOT_READY_MODE if current mode is ROBOT_RUNNING_MODE
   */
  void robotinterface_set_robot_ready_mode();

  /**
   * Sets the robot in ROBOT_RUNNING_MODE if current mode is
   * ROBOT_READY_MODE or ROBOT_FREEDRIVE_MODE
   *
   */
  void robotinterface_set_robot_running_mode();

  /**
   * Sets the robot in ROBOT_FREEDRIVE_MODE if current mode is
   *  ROBOT_RUNNING_MODE
   *
   */
  void robotinterface_set_robot_freedrive_mode();


  /**
   * Returns the state of one joint.
   *
   * Each joint can be in one of the
   * following states.
   *
   * - JOINT_MOTOR_INITIALISATION_MODE
   * - JOINT_BOOTING_MODE
   * - JOINT_DEAD_COMMUTATION_MODE
   * - JOINT_BOOTLOADER_MODE
   * - JOINT_CALIBRATION_MODE
   * - JOINT_STOPPED_MODE
   * - JOINT_FAULT_MODE
   * - JOINT_RUNNING_MODE
   * - JOINT_INITIALISATION_MODE
   * - JOINT_IDLE_MODE
   */
  uint8_t robotinterface_get_joint_mode(int joint);

  /**
   * Returns the state of the tool
   *  Either:
   * - JOINT_IDLE_MODE
   * - JOINT_BOOTLOADER_MODE
   * - JOINT_RUNNING_MODE
   */
  uint8_t robotinterface_get_tool_mode();

  /**
   * Returns number_of_message that needs to be read by
   * #robotinterface_get_next_message().
   */
  uint8_t robotinterface_get_message_count();

  /* Revisit this list when making a new logger system */
  /* Source is:
   69: Safety sys 2
   68: Euromap67 2
   67: Euromap67 1
   66: Teach Pendant 2
   65: Teach Pendant 1
   6: Tool
   5: Wrist 3
   4: Wrist 2
   3: Wrist 1
   2: Elbow
   1: Shoulder
   0: Base
  -1: Safetysys (errors reported by masterboard code
  -2: Controller
  -3: RTMachine
  -4: Simulated Robot
  -5: GUI
  -6: ControlBox (not used, was used in a Version mismatch check in the controller, the source for that message has been replaced with -2)
  */

  struct message_t {
    uint64_t timestamp;
    char source;
    char *text;
  };

  /**
   * Returns length of message copied to the char*
   */
  int robotinterface_get_message(struct message_t *message);

  /**
   * Takes a message as error codes rathern than text
   */
  int robotinterface_get_message_codes(struct message_t *msg, int *error_code,
                                       int *error_argument);

  /*@}*/

  /**
   * \name Security stops
   *
   * This is where the robot security stop methods start.
   * <em>Implemented in sequritycheck.c</em>
   *@{
   */

  /**
   * \return 0 if there is no error in the robot, true otherwise.
   */
  void robotinterface_power_on_robot();
  void robotinterface_power_off_robot();
  void robotinterface_security_stop(char joint_code, int error_state,
                                    int error_argument);
  int robotinterface_is_power_on_robot();
  int robotinterface_is_security_stopped();
  int robotinterface_is_emergency_stopped();
  int robotinterface_is_extra_button_pressed(); /* The button on the back side of the screen */
  int robotinterface_is_power_button_pressed();  /* The big green button on the controller box */
  int robotinterface_is_safety_signal_such_that_we_should_stop(); /* This is from the safety stop interface */
  int robotinterface_unlock_security_stop();
  int robotinterface_has_security_message();
  int robotinterface_get_security_message(struct message_t *message,
                                          int *error_state,
                                          int *error_argument);
  uint32_t robotinterface_get_master_control_bits();

  /*@}*/



  /**
   * \name Robot methods
   *
   * This is where actual robot methods start.
   *
   * Each function copies the resulting joint values of the
   * robot to the supplied array.
   *@{
   */
  void robotinterface_get_actual(double *q, double *qd);
  void robotinterface_get_actual_position(double *q);
  void robotinterface_get_actual_velocity(double *qd);
  void robotinterface_get_actual_current(double *I);
  void robotinterface_get_tool_accelerometer_readings(double *ax, double *ay,
                                                double *az);
  double robotinterface_get_tcp_force_scalar();
  void robotinterface_get_tcp_force(double *F);
  void robotinterface_get_tcp_speed(double *V);
  double robotinterface_get_tcp_power();
  double robotinterface_get_power();

  /*@}*/

  int robotinterface_get_actual_digital_input(int port);
  int robotinterface_get_actual_digital_output(int port);
  unsigned short robotinterface_get_actual_digital_input_bits();
  unsigned short robotinterface_get_actual_digital_output_bits();
  double robotinterface_get_actual_analog_input(int port);
  double robotinterface_get_actual_analog_output(int port);
  unsigned char robotinterface_get_actual_analog_input_range(int port);
  unsigned char robotinterface_get_actual_analog_output_domain(int port);

  /** \name Ideal methods
   * This is where target (ideal) methods start.
   */
  /*@{*/
  void robotinterface_get_target(double *q, double *qd, double *qdd);
  void robotinterface_get_target_position(double *q);
  void robotinterface_get_target_velocity(double *qd);
  void robotinterface_get_target_acceleration(double *qdd);
  void robotinterface_get_target_current(double *I);
  void robotinterface_get_target_moment(double *m);

  unsigned short robotinterface_get_target_digital_output_bits();
  double robotinterface_get_target_analog_output(int port);
  unsigned char robotinterface_get_target_analog_input_range(int port);
  unsigned char robotinterface_get_target_analog_output_domain(int port);

  /*@}*/


  /**
   *
   * \name Temperature, Voltage and Current Methods
   */
  /*@{*/

  /* master */
  float robotinterface_get_master_temperature();  /* new */
  float robotinterface_get_robot_voltage_48V(); /* new */
  float robotinterface_get_robot_current(); /* new */
  float robotinterface_get_master_io_current(); /* new */
  unsigned char robotinterface_get_master_safety_state();
  unsigned char robotinterface_get_master_on_off_state();
  void robotinterface_safely_remove_euromap67_hardware();
  void robotinterface_disable_teach_pendant_safety();

  /* joint */
  void robotinterface_get_motor_temperature(float *T);
  void robotinterface_get_micro_temperature(float *T);
  void robotinterface_get_joint_voltage(float *V);

  /* tool */
  float robotinterface_get_tool_temperature();
  float robotinterface_get_tool_voltage_48V();
  unsigned char robotinterface_get_tool_output_voltage();
  float robotinterface_get_tool_current();

  /* Euromap for CB2.0*/
  uint8_t robotinterface_is_euromap_hardware_installed();
  uint8_t robotinterface_get_euromap_input(uint8_t input_bit_number);
  uint8_t robotinterface_get_euromap_output(uint8_t output_bit_number);
  void robotinterface_set_euromap_output(uint8_t output_bit_number, uint8_t desired_value);
  uint32_t robotinterface_get_euromap_input_bits();
  uint32_t robotinterface_get_euromap_output_bits();
  uint16_t robotinterface_get_euromap_24V_voltage();
  uint16_t robotinterface_get_euromap_24V_current();

  /* General purpose registers */
  uint16_t robotinterface_get_general_purpose_register(int address);
  void robotinterface_set_general_purpose_register(int address, uint16_t value);
  void robotinterface_send_joint_regulation_parameter(int joint_id, int param_id, int param_value);
  void robotinterface_send_joint_special_parameter(int joint_id, int param_id, int param_value);

  /*@}*/


#ifdef __cplusplus
}
#endif
#endif        /* _ROBOTINTERFACE_H */
