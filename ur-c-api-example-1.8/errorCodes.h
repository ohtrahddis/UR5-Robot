#ifndef __ERRORCODES_H_
#define __ERRORCODES_H_

#define NO_ERROR 0
#define OUTBUFFER_OVERFLOW_ERROR 1
#define INBUFFER_OVERFLOW_ERROR 2
#define PROCESSOR_OVERLOADED_ERROR 3
#define BROKEN_COMMUNICATION_ERROR 4
/*Arguments from controlbox and nodes on secondary BUS:
   1: Communication with PC lost.
   2: Communication with primary   MASTERBOARD uP lost
   3: Communication with secondary MASTERBOARD uP lost
   4: Communication with primary   SCREEN uP lost
   5: Communication with secondary SCREEN uP lost
   6: Communication with primary   EUROMAP67 uP lost
   7: Communication with secondary EUROMAP67 uP lost
   8: Communication with primary   EUROMAP67 uP present, but euromap67 is disabled 
   9: Communication with secondary EUROMAP67 uP present, but euromap67 is disabled
  10: Communication with primary   Teach Pendant present, but Teach Pendant safety is disabled 
  11: Communication with secondary Teach Pendant uP present, Teach Pendant safety is disabled
  65: Lost package from Primary   Screen
  66: Lost package from Secondary Screen
  67: Lost package from Primary   Euromap67
  68: Lost package from Secondary Euromap67
  69: Lost package from Secondary Masterboard
*/
#define HEAVY_PROCESSOR_LOAD_WARNING 5
#define MOTOR_ENCODER_INDEX_MOUNTING_OR_INTERRUPT_DRIFT_ERROR_DETECTED 6
#define JOINT_ENCODER_INDEX_INTERRUPT_DRIFT_ERROR_DETECTED 7
#define JOINT_ENCODER_INDEX_MIRROR_DETECTED 8 /* was ENCODER_INDEX_DRIFTED_ERROR */
#define OUTBUFFER_NOT_FULLY_SENT_ERROR 9
#define BROKEN_PC_COMMUNICATION_ERROR 10
#define BAD_CRC_ERROR 11
#define UNKNOWN_MESSAGE_ERROR 12
#define SEND_TO_ROBOT_FAILED_TO_COMPLETE_IN_TIME 13
#define DEBUG_ERROR 14
#define MASTER_SNIFFED_MESSAGE_ADRESSED_TO_INVALID_NODE_ID 15
#define MESSAGE_TYPE_DOES_NOT_MATCH_AMOUNT_OF_DATA 16
#define INBUFFER_OVERFLOW_ERROR_IN_MASTER_FROM_PC 17
#define MASTER_HAD_EXTRA_DATA_IN_BUFFER_FROM_PC 18
#define MASTER_HAD_NO_DATA_FROM_PC_TO_SEND_TO_JOINTS 19
#define FATAL_POWERBRIDGE_FAULT 20
#define ENCODER_JOINT_INTERRUPT_WITHOUT_CHANGE_ERROR 21
#define ENCODER_JOINT_INTERRUPT_WITH_TWO_STEP_CHANGE_ERROR 22
#define ENCODER_MOTOR_INTERRUPT_WITHOUT_CHANGE_ERROR 23
#define ENCODER_MOTOR_INTERRUPT_WITH_TWO_STEP_CHANGE_ERROR 24
#define JOINT_ENCODER_INDEX_DRIFT_DETECTED 25
#define MOTOR_ENCODER_INDEX_DRIFT_DETECTED 26
#define STRANGE_INTERRUPT_ERROR 27
#define DATA_AVAILABLE_BITS_WERE_ALL_ZERO 28
#define ETHERNET_PACKAGE_LOSS_DETECTED_FROM_PC_TO_MASTER 29
#define MASTER_RECIEVED_DATA_FROM_TOO_MANY_JOINTS_ERROR 30
#define JOINT_HAS_CAUGT_WRONG_MESSAGE_FROM_MASTER 31
#define FLASH_WRITE_VERIFY_FAILED 32
#define CALIBRATION_FLASH_CHECKSUM_FAILED 33
#define CALIBRATION_DATA_OUT_OF_BOUNDS 34
#define JOINT_ID_IS_UNDEFINED 35
#define ILLEGAL_BOOTLOADER_COMMAND 36
#define INBUFFER_PARSE_ERROR 37
#define MASTER_GOT_JOINT_MESSAGES_IN_THE_WRONG_ORDER 38
#define COMMUNICATION_ANOMALY 39
#define ADCONVERTER_HIT_HIGH_LIMIT 40
#define CURRENT_REGULATION_INTEGRAL_PART_RUNAWAY_ERROR 41
#define MAX_MOTOR_SPEED_REACHED 42
#define JOINT_SERVO_DELTA_POSITION_TOO_LARGE 43
#define CRC_ERROR_POSSIBLY_FROM_JOINT 44
#define AD_CONVERTER_ERROR 45
#define BAD_ENCODER_MOUNTING_OR_GEARBOX_BROKEN_ERROR 46
#define ADCONVERTER_HIT_LOW_LIMIT 47
#define VOLTAGE_DROP_DETECTED 48
#define RS485_RECIEVE_ERROR 49
/*Arguments from controlbox:
 200: Secondary RS485 bus is down.
*/
#define ROBOT_POWERUP_FAILURE 50
/*Arguments from controlbox:
   1: (OLD) The 24V I/O has voltage, but 24V regulator is off
   2: Voltage present at unpowered robot (OLD Robot voltage is not off after powerup)
   4: (OLD) Robot is not stopped, this is an error (Old Emergency Stop relays are on without 24V power!)
   5: 48V PSU voltage to low after powerup
   6: 48V PSU voltage to high after powerup
  11: (OLD) Voltage not detected at 24V rail after startup
  15: Warning, Waiting for secondary masterboard processor
  16: Warning, Waiting for screen (Check cable)
  17: Warning, Waiting for euromap67 interface (Check ribbon cable)
  18: Warning, Waiting for primary masterboard processor
  19: Warning, Waiting for a valid "euromap67 activated" status bit from secondary masterbaord processor
  20: 5V measured too high, 5V, 3V3 or ADC malfunction
  21: 5V measured too low, 5V, 3V3 or ADC malfunction
  22: Robot current sensor output to high for offset calibration
  23: Robot current sensor output to low for offset calibration
  24: 48V PSU voltage not preset. Check internal connection. (Maybe it does not turn on or maybe it has reverse polarization) 
  25: Robot voltage preset when the 48V PSU is powered up. (Transistors ON/OFF function might be short circuited)
  26: Voltage present on unpowered 48V PSU
  27: 12V measured too high, 12V, 3V3 or ADC malfunction
  28: 12V measured too low, 12V, 3V3 or ADC malfunction
  29: -12V measured too high, Analog I/O malfunction
  30: -12V measured too low, Analog I/O malfunction
  31: The other SafetySys will not initialize. (Or euromap67, one or both masterboard uP will not initialize)
  40: # Wrong voltage from PSU1
  41: # Wrong voltage from PSU2
  42: # Voltage will not disappear from PSU
  43: # Warning, waiting for CB2 type answer from primary processor
  99: Wrong software on PCB
 100: Cable not connected
 101: Short Circuit in robot
 102: Voltage rising too slow
 103: Voltage did not reach appropriate level (E.g 48V != 48V)
*/
#define EMERGENCY_RELAY_FAILURE 51
#define EMERGENCY_STOP_BUTTON_PRESSED 52
#define IO_OVERCURRENT_DETECTED 53
#define POSSIBLE_SHORT_CIRCUIT_AT_24V_DETECTED 54 /* Not used. */
#define SAFETY_CIRCUIT_ABNORMAL_OPERATION 55      /* same as FATAL_SAFETY_ERROR 55 */
#define FATAL_SAFETY_ERROR 55
/*        Arguments from controlbox:
         11:  (OLD) Stop indication and not emergency stop indication
         12:  (OLD) Robot voltage indic. error and not emergency stop indication
         16:  (OLD) Stop indication is low but should be high
         17:  (OLD) safetyErrorIntegratorB>=6000000
         23:  Safety Relay Output Error, malfunction on secondary uP control (minus connection)  (also OLD safety relay error)
         24:  Safety Relay Output Error, malfunction on ColdFire uP control (plus connection)    (also OLD safety relay error)
         33:  Safety Relay Output Error, Relay is stock (in E67 module a reset on one uP will force the MAF-relay check signal to be high, and this error will appear)  (also OLD safety relay error)
         34:  Safety Relay Output Error, Relays are off for no reason                           (also OLD safety relay error)
         50:  Voltage present on unpowered robot (Normally not possible when the communication with the joints are present, but if it is not, it is possible to backdrive the robot very fast because the H-bridge is none-active)
         51:  Voltage will not disappear from robot
         52:  5V measured to low, 5V, 3V3 or ADC malfunction
         53:  5V measured to high, 5V, 3V3 or ADC malfunction
         90:  Bootloader error, voltage too low (current might be too high)
         91:  Bootloader error, robot voltage higher than 48V + tolerance
         100: Safety violation (Impossible Software Error)
         101: Safety Channel Error In Masterboard (States are different)
         102: Safety Channel Error In Screen (States are different)
         103: Safety Channel Error In Euromap67 Interface (States are different)
         109: Received FAULT message from PC.
         110: Safety State is changing too often
         111: On/Off State is changing too often
         112: The two current sensors are too different
         120: Robot current is too high in emergency stop state
         121: Robot current is too high in safeguard stop state
         130: The secondary uP is FAULT
         131: At least one screen uP is FAULT
         132: At least one euromap67 uP is FAULT
         
         Arguments from joints
         52:  5V measured to low, 5V, 3V3 or ADC malfunction
         53:  5V measured to high, 5V, 3V3 or ADC malfunction
         
         */
#define OVERVOLTAGE_SHUTDOWN 56
#define JOINT_SERVO_DELTA_SPEED_TOO_LARGE 57
#define MOTOR_ENCODER_NOT_CALIBRATED 58
         /*
         0: Never calibrated
         1: Wrongly calibrated, or joint could not decelerate well enough after motor init.
         */
#define OVERCURRENT_SHUTDOWN 59
#define ROBOT_POWER_OFF_AT_HIGH_CURRENT 60
#define MISSED_JOINT_INDEX_MARK 61
        /*   Arguments:
         *    11: Index mark could not be found during first five minutes of self-test
         *    20: Index mark could not be found during joint part d calibration
         */
#define THERMAL_SHUTDOWN 62
#define MOTOR_TEST_FAILED 63
        /*  Arguments:
         *    The number telling which step the joint test failed in
         */
#define INTERFACE_CONNECTION_ERROR 64
/*       Arguments from controlbox:
         1:   Safety error: Auto is active but reset is not.
         2:   Safety error: Reset button is stock
         3:   I/O overcurrent at 24V, max is 1200mA
         4:   I/O overcurrent at 24V, max is 2A*/
#define REBOOT_WARNING 65
/*       Arguments from controlbox:
         0:     "no argument" (leaving fault or bootloader mode etc. - or "I am rebooting because the other uP is)
         1-199: Rebooting for the XX time.
         200:   Leaving fault
         201:   PC forced a reboot
         202:   I reboot because the other uP does.
*/
#define VERSION_MISMATCH_ERROR 66
#define CONFIGURATION_WARNING 67
/*        Arguments from controlbox:
         0:   "no argument"
         1:   Euromap67 safety installed
         2:   Euromap67 safety uninstalled
         3:   Teach Pendant safety installed
         4:   Teach Pendant safety uninstalled
         5:   Error, cannot uninstall a none-installed interface
         6:   Booting for the first time
         */


#define SPI_ERROR 68
#define SEND_ERRORS_ACKNOWLEDGE_FROM_MASTER_TO_CONTROLLER 69 /* Used for handshake between master and controller when the controller wants the master to send its errors */
#define CLOSE_TO_GEARBOX_BROKEN_ERROR_WARNING 70
#define JOINT_STARTUP_CHECK_ERROR 71
/* Args:
        1 = Joint hardware is size 1 but software is not
        2 = Joint hardware is size 2 but software is not
        3 = Joint hardware is size 3 but software is not
        4 = Joint hardware is size 4 but software is not
        5 = Wrong hardware size read
        6 = AD Converters for motor current not working
        7 = Invalid value of the motor test results variable (>15, four bits are used)
        8 = Motor short circuit to ground (motor indication signal is not high when all PWM outputs are low)
        9 = Error on either motor phase 1 or phase 3
        10 = Error on motor phase 2
        11 = Error in connection between phase 3 and motor indication signal
        12 = Error combination 1
        13 = Error combination 2
        14 = Error combination 3
        15 = Error combination 4
*/


#define NOTIFICATION_MESSAGE 72 /* This error code can be used to notify about some incident that is not an error */
/* Args.:
 *        1 = Masterboard recognized it was placed in a UR5 controller box (Message = Starting UR5)
 *        2 =                                           UR10               (Message = Starting UR10)
 *
 *
 */
#define BRAKE_TEST_FAILED 73

#define JOINT_ENCODER_WARNING 74
 /* Args as bit field:
  *   2 : Speed not valid 
  *   8 : Supply voltage is out of range
  *  16 : Temperature is out of range
  *  64 : Signal amplitude too low : Too far from magnetic ring
  * 128 : Signal amplitude too high : Too close to magnetic ring or external magnetic field present
  */
#define JOINT_ENCODER_ERROR 75
 /* Args as bitfield:
  *   1 : Invalid decoding (Module not aligned / ring is damaged / external magnetic field / acceration too high)
  *   4 : System error (Malfunction detected inside circuitry or inconsistent calibration data is detected)
  *  32 : Signal lost (Misaligned readhead or damaged magnetic ring)
  */
#define JOINT_ENCODER_COMMUNICATION_ERROR 76

#define JOINT_ENCODER_DELTAPOS_ERROR 77
#define JOINT_ENCODER_DELTAPOS_DISCARD_ERROR 78

/*  
 * Arg = Joint_encoder position delta value (max 0xff)
 */

/* These errors come from robotinterface. */
#define ROBOT_MODE_CHANGED 100
#define REAL_ROBOT_CONNECTED 101
#define REAL_ROBOT_NOT_CONNECTED 102
#define UR_ETHERNET_ERROR 103
#define NO_COMMAND_SENT_TO_ROBOT_ERROR 104
#define JOINT_NOT_RESPONDING 105
#define ALL_JOINTS_NOT_RESPONDING 106
#define JOINT_HAD_TOO_MANY_ERRORS_IN_A_ROW 107
#define ALL_JOINTS_HAD_TOO_MANY_ERRORS_IN_A_ROW 108
#define SECURITY_STOP 109
#define JOINT_SECURITY_STOP 110
#define SOMETHING_IS_PULLING_THE_ROBOT 111
#define LARGE_POSITION_CHANGE_AT_STARTUP 112
#define FORCE_AND_POWER_PROTECTIVE_STOP 113
#define HIGH_FORCE_AND_POWER_WARNING 114
#define WRONG_ROBOT_TYPE 115
#define REALTIME_PART_TOOK_TO_LONG 116

/* These errors come from securitycheck */
#define SECURITY_CHECK_DELTA_POSITION_VIOLATION 150
#define SECURITY_CHECK_POSITION_LIMIT_VIOLATION 151
#define SECURITY_CHECK_DELTA_SPEED_VIOLATION 152
#define SECURITY_CHECK_VELOCITY_LIMIT_VIOLATION 153
#define SECURITY_CHECK_VELOCITY_DIR_VIOLATION 154
#define SECURITY_CHECK_ACCELERATION_DIR_VIOLATION 155
#define SECURITY_CHECK_TORQUE_LIMIT_VIOLATION 156

/* These errors come from the controller */
#define COLLISION_DETECTED 170
#define RUNTIME_ERROR 171
/*
 * Args:
 *  0. Runtime machine caught: RuntimeException
 *
 */
#define ILLEGAL_CONTROL_MODE 172
#define JOINT_LIMIT_DETECTED 173
#define SINGULARITY_DETECTED 174 /* TODO: This error is not used. I think we should delete it to allow alternative usage /Emil */

/* These error come from calibration */
#define CALIBRATION_CHECKSUM_A_FAILED 180
#define CALIBRATION_CHECKSUM_C_FAILED 181
#define NO_CALIBRATION_FILE_FOUND 182
#define CALIBRATION_ANOMALY 183
#define SELFTEST_NOT_COMPLETED 184

/* Modbus communication errors */
#define MODBUS_ERROR 190

/* Force mode errors*/
#define FORCE_MODE_ERROR 191
/*
 * Args:
 *  1. Robot too close to singularity
 *  2. Distance from tcp to feature too small for force mode type 1
 *  3. Tool speed too low for force mode type 3
 *  4. Invalid force mode type
 *  5. Position error too large
 *  6. Orientation error too large
 *  7. Joint firmware too old for force mode
 */
 
 /* Eccentricity calibration error*/
#define ECC_CALIB_ERROR 192
/*
 * Args:
 *  1. Calibration has been restarted because it has iterate too many times
 *  2. Calibration has been restarted because the average error is too large
 *  3. The estimated parameters is too large, verify the hardware and restart the selftest
 *  4. The verification of the average error faild.
 *  10-11. Code failure!
 */

#endif
