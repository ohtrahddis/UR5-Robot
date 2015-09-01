#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

#include <stdint.h>

struct ForceConfiguration {
    double p_gain_motion_translation;
    double p_gain_motion_orientation;
    double d_gain_motion_translation;
    double d_gain_motion_orientation;
    double p_gain_force_translation;
    double p_gain_force_orientation;
    double force_error_damping;
    double compliant_axes_velocity_damping_translation;
    double compliant_axes_velocity_damping_orientation;
    double compliant_axes_speed_limit_gain_translation;
    double compliant_axes_speed_limit_gain_orientation;
    double stiction_gain;
    double joint_softness;
    double minimum_dexterity;
    double minimum_distance_to_point;
    double minimum_tcp_velocity;
    double make_print_to_file;
};

struct JointConfiguration {

  int id;

  /* [Joint] section */
  double min_limit;
  double max_limit;
  double v_max;
  double a_max;
  double torque_max;
  double gear_ratio;
  double current_measurement_error_window;
  double temperatur_model_parameters[4];
  double teach_stiffness;
  double v_max_teach_mode;

  /* Legacy friction model parameters start */
  double static_friction;
  double dynamic_friction;
  double viscous_friction;
  double dynamic_friction_backdrive;
  double viscous_friction_backdrive;
  double dynamic_correction_term;
  double dynamic_correction_term_backdrive;
  double stiction_positive_qd_calculated_from_selftest;
  double stiction_negative_qd_calculated_from_selftest;
  double friction_constant_calculated_from_selftest;
  double friction_constant_correction_term;
  double friction_constant_correction_term_backdrive;
  /* Legacy friction model parameters end */

  /* [Motor] section */
  double rotor_inertia;
  double torque_constant;
  double torque_constant_calculated_from_selftest;
  double winding_resistance;
  int motor_type; /* 1 == kreipke, 2 == kollmorgen 3 == sparePartCB3->CB2Kollmorgen */

  /* [Units] section */
  int JOINT_ENCODER_TICKS_PER_REVOLUTION;
  double APOS_U_PER_JOINT_RADIAN;
  double AVEL_U_PER_JOINT_RADIANS_PER_SECOND;
  double CUR_U_PER_AMP;
  double VOL_U_PER_VOLT;
  double MAX_CURRENT;
  double MOTOR_TEMP_U_PER_DEGREE;
  double MOTOR_TEMP_OFFSET;
  
};

/* This should be part of the joint struct */
struct Calibration {

  int joint_revision[6];  /* 1 == Version 1.3 / Controllerbox 2.0*/
  int calibration_status;  /* 0 == notInitialized / 1 == notLinearised / 2 == Linearised */
  uint32_t joint_checksum[6];

  double delta_a[6];
  double delta_d[6];
  double delta_alpha[6];
  double delta_theta[6];
  double encoder_sin_excentricity[6];
  double encoder_cos_excentricity[6];

  /* Test results from motor calibration */
  int calibrationDataReadyToBeWrittenToFile;
  int done;

  /* Self-test data from here */
  double self_test_measurement_speed[8][6];
  double self_test_measurement_voltage[8][6];
  double self_test_measurement_current[8][6];
  double self_test_measurement_temp[8][6];
  int self_test_completed[6];
  /* Self-test data to here */
};

struct Configuration {

  /* configuration-file path */
  char *path;

  /* JointConfiguration */
  struct JointConfiguration joint[6];

  /* [Joint] section */
  double v_joint_default;
  double a_joint_default;

  /* [Tool] section */
  double tcp_pose[6];
  double tcp_payload_mass;
  double tcp_payload_cog[3];
  double v_tool_default;
  double a_tool_default;
  double eqradius;

  /* [Link] section */
  double mass[6];
  double center_of_mass[6][3];
  double inertia_matrix[6][3][3];
  double gravity[3];

  /* [DH] section */
  double a[6];
  double d[6];
  double alpha[6];
  double q_home_offset[6];
  int joint_direction[6];

  /* [Units] */
  double MASTER_POWERBUS_VOLTAGE_UNITS_PER_VOLT;
  double MASTER_POWERBUS_CURRENT_UNITS_PER_AMP;
  double MASTER_POWERBUS_CURRENT_UNITS_OFFSET;
  double MASTER_TEMP_UNITS_PER_DEGREE;
  double MASTER_TEMP_UNITS_OFFSET;
  double MASTER_IO_CURRENT_UNITS_PER_AMP;
  double MASTER_5V_VOLTAGE_UNITS_PER_VOLT;
  double TOOL_ACCELEROMETER_UNITS_PER_M_S_S;
  double TOOL_ACCELEROMETER_OFFSET_UNITS;

  /* [Config] section */
  int masterboard_version;
  int dump_bytecode_on_exception;
  int ignore_missing_joints; /* Not set in urcontrol.conf per default */

  /* [Hardware] section */
  int controller_box_type; /* Tells what kind of controller box the controller is running in. (1=CB1 (old white cabinet with emergency stop button), 2=CB2) */
  int robot_type; /* Tells what kind of robot the controller is connected to. (1=UR5, 2=UR10) */
  int robot_sub_type; /* May be used to differentiate between hardware versions */

  /* Does not appear in urcontrol.conf */
  int masterboard_detected_version; /* Send by masterboard */
  int masterboard_detected_controllerbox_type; /* Send by masterboard */

  struct Calibration calibration;
  struct ForceConfiguration force;

  int reloadConf;
};

extern struct Configuration conf;

#ifdef __cplusplus
extern "C" {
#endif

  int calibration_load();
  int calibration_save();
  int configuration_load();
  void check_for_valid_hw_combo();
  void configuration_print();
  void calibration_print(int);
  const char* getJointVersion(int const jointId);
  void setForceParamsToDefault();

#ifdef __cplusplus
}
#endif

#endif              /* _CONFIGURATION_H */
