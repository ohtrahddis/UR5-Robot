#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

struct JointConfiguration {

     /* [Joint] section */
     double min_limit;
     double max_limit;
     double v_max;
     double a_max;
     double torque_max;
     double torque_max_avg;
     double gear_ratio;
     double static_friction;
     double dynamic_friction;
     double viscous_friction;
     /* double columb_friction; */
     double dynamic_friction_backdrive;
     double viscous_friction_backdrive;
     /* double columb_friction_backdrive; */
     double freedrive_dynamic_fraction;
     double freedrive_viscous_fraction;

     double current_measurement_error_window;

     /* [Motor] section */
     double rotor_inertia;
     double torque_constant;

     /* [Units] section */
     int JOINT_ENCODER_TICKS_PER_REVOLUTION;
     double APOS_U_PER_JOINT_RADIAN;
     double AVEL_U_PER_JOINT_RADIANS_PER_SECOND;
     double CUR_U_PER_AMP;
     double MAX_CURRENT;
     double ACCELEROMETER_UNITS_PER_M_S_S;
     double ACCELEROMETER_OFFSET_UNITS;
};

struct Calibration {
     int joint_revision[6];  /* 1 == Version 1.3 / Controllerbox 2.0*/

     double delta_a[6];
     double delta_d[6];
     double delta_alpha[6];
     double delta_q_home[6];
     double encoder_sin_excentricity[6];
     double encoder_cos_excentricity[6];

     /* Test results from motor calibration */
     double voltage_low_speed_front[6];
     double voltage_high_speed_front[6];
     double voltage_low_speed_back[6];
     double voltage_high_speed_back[6];
     double current_low_speed_front[6];
     double current_high_speed_front[6];
     double current_low_speed_back[6];
     double current_high_speed_back[6];
     double temp_low_speed_front[6];
     double temp_high_speed_front[6];
     double temp_low_speed_back[6];
     double temp_high_speed_back[6];
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
     double v_tool_default;
     double a_tool_default;
     double eqradius;

     /* [Link] section */
     double mass[6];
     double center_of_mass[6][3];
     double gravity[3];

     /* [DH] section */
     double a[6];
     double d[6];
     double alpha[6];
     double q_home_offset[6];

     /* [Units] */
     double TOOL_ACCELEROMETER_UNITS_PER_M_S_S;
     double TOOL_ACCELEROMETER_OFFSET_UNITS;
     double MASTER_POWERBUS_VOLTAGE_UNITS_PER_VOLT;
     double MASTER_POWERBUS_CURRENT_UNITS_PER_AMP;
     double MASTER_POWERBUS_CURRENT_UNITS_OFFSET;
     double MASTER_TEMP_UNITS_PER_DEGREE;
     double MASTER_TEMP_UNITS_OFFSET;
     double MASTER_IO_CURRENT_UNITS_PER_AMP;
     double MASTER_5V_VOLTAGE_UNITS_PER_VOLT;

     /* [Config] section */
     int masterboard_version;

     int ignore_missing_joints;

     struct Calibration calibration;

};

extern struct Configuration conf;

#ifdef __cplusplus
extern "C" {
#endif

     int configuration_load();
     void configuration_close();
     void configuration_print();

#ifdef __cplusplus
}
#endif
#endif              /* _CONFIGURATION_H */
