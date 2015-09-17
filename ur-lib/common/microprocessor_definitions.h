/*### Errata for preprocessors and compilers ###
1) Preprocessor for MC56F8323 - Left shift with more than 14 does not work! i.g. (1<<15) will not work!

*/

#ifndef __MICROPROCESSOR_UTILITIES_H_
#define __MICROPROCESSOR_UTILITIES_H_

/*********************************************************************************************************
* Configuration definitions (Could be adjusted)
*********************************************************************************************************/
#define MAX_SAFETY_STOP_TIME                                          1000       /* [ms] The robot must stand still after this many ms when it is emergency stopped or safeguard stopped. */

#ifdef MASTERBOARD_PRIMARY
#define TF                                                            2         /* [] TF = Time Factor (1ms/TF = doStuff-tick-frequency for primary masterboard uP (ColdFire)) */
#endif
#ifdef MASTERBOARD_SECONDARY
#define TF                                                            2         /* [] TF = Time Factor (1ms/TF = doStuff-tick-frequency for secondary masterboard uP) */
#endif
#ifdef SCREEN
#define TF                                                            2         /* [] TF = Time Factor (1ms/TF = doStuff-tick-frequency for euromap67 uPs) */
#endif
#ifdef EUROMAP67
#define TF                                                            2         /* [] TF = Time Factor (1ms/TF = doStuff-tick-frequency for screen uPs) */
#endif
#define MASTERBOARD_MAX_SAFETY_STOP_TIME               (TF*MAX_SAFETY_STOP_TIME)/* [ms] (32bit) The maximum stop time of the robot when an emergency stop or safeguard stop is issued. After this time the current for the robot must be lower than a sudden level */
#define MASTERBOARD_ETHERNET_STABILIZING_TIME                 (TF*    500      )/* [ms] (32bit) The stabilizing time for the ethernet communication. Time from first message to communication approved. Without this time, the safety code might start up and then reboot shortly after. It seems as if the first "hole" in the communication happens ~350ms after first ethernet package*/
#define MASTERBOARD_24V_REGULATOR_STABILIZING_TIME            (TF*    200      )/* [ms] (32bit) The time that the 24V regulator needs to stabilize its output voltage. This time must be long enouth to ensure that a 24V I/O voltage measurement is transmittet by the cyclic input from the secondary uP*/
#define MASTERBOARD_INITIALIZATION_WAIT_TIMEOUT               (TF*    90000    )/* [ms] (32bit) When a masterboard uP goes in initialization state it waits for the other masterboard uP to do the same (or E67 waits for masterbaord uPs). This is the timeout before generating an error/warning. The time must be long enough to wait for the pc to start up (~80sec @ long DHCP search. The GUI is ready after ~88sec @ long DHCP search. ~35sec/52sec with ethernet connection.)*/
#define MASTERBOARD_LOOK_FOR_INTERFACE_TIME                   (TF*    100      )/* [ms] (32bit) Look for euromap67 or teach pendant for this long when the interface is disabled. If found and not enabled then store the euromap67 or teach pendant bit in the flash. */
#define MASTERBOARD_EXPECT_BUSNODE_BEFORE_TIME                (TF*    500      )/* [ms] (32bit) Expect RS485 communication with safety-units before x ms after power-up */
#define MASTERBOARD_VOLTAGE_DISAPPEARING_TIMEOUT              (TF*    12000    )/* [ms] (32bit) Max time to wait for 48V PSU voltage or robot voltage to be < MASTERBOARD_MAX_ROBOT_VOLTAGE_AT_POWERUP */
#define MASTERBOARD_TIME_TO_CALIBRATE_ROBOT_CURRENT_SENSOR    (TF*    100      )/* [ms] (32bit) Time between beginning of INITIALIZATION (both processors) to the 48V PSU is turned on */
#define MASTERBOARD_TIME_BETWEEN_PSUS                         (TF*    300      )/* [ms] (32bit) Time between 48V PSU1 power-up to 48V PSU2 power-up */
#define MASTERBOARD_TIME_TO_STABILIZE_48V_PSU                 (TF*    1500     )/* [ms] (32bit) Set and rise time for the 48V PSUs */
#define MASTERBOARD_SAFETY_IO_FILTER_TIME                     (TF*    50       )/* [ms] (32bit) Safety I/O must have a stady-state for this long when going down in safety level priority (This time must be long enough to ensure that every uP on the safety BUS gets a message where the safety state is changed in - also when there is some package losses (Therefor min 25ms)) */
#define MASTERBOARD_SAFEGUARD_RESET_STOCK_BUTTON_TIMEOUT      (TF*    2000     )/* [ms] (32bit) The longest time that the reset button may be pressed */
#define MASTERBOARD_SAFETY_STATE_DIFFERENT_TIMEOUT            (TF*    500      )/* [ms] (32bit) Safety states may not be differet for longer time than this constant (This is the most important safety constant!) */
#define MASTERBOARD_SAFETY_STATE_CHANGE_NOTICE_LIFETIME       (TF*    100      )/* [ms] (32bit) [NOTE! - MASTERBOARD_SAFETY_STATE_CHANGE_NOTICE_LIFETIME / MASTERBOARD_MAX_SAFETY_STATE_CHANGES_WITHIN_LIFETIME must be an interger] How often the SAFETY_STATE may change is defined by these two numbers devided like that. */
#define MASTERBOARD_ROBOT_ON_OFF_CHANGE_NOTICE_LIFETIME       (TF*    1000     )/* [ms] (32bit) Same consept as with the safety state */
#define MASTERBOARD_SOFTWARE_RESET_HOLD_TIME                  (TF*    3000     )/* [ms] (32bit) Stay in booting state for this long, if a software reset was generated (safety state changed back to booting). This time should be long enough to ensure that the voltage on 48V PSU is ~0V */
#define MASTERBOARD_FAULT_REBOOT_HOLD_TIME                    (TF*    10000    )/* [ms] (32bit) Stay in booting state for this long, before waiting for a reboot event from the PC */
#define MASTERBOARD_MAX_24V_CURRENT_EXCEED_TIME               (TF*    500      )/* [ms] (32bit) The 24V current may be overloadet for this long lime. */
#define MASTERBOARD_MAX_24V_CURRENT_RETRY_TIME                (TF*    10000    )/* [ms] (32bit) The 24V is turned on again after a short circuit after this long time */
#define MASTERBOARD_MAIN_POWER_OFF_TIMEOUT                    (TF*    300      )/* [ms] (32bit) The 12V PC current must be lower the thresshold for this long time before the power off will take place */
#define MASTERBOARD_ROBOT_CURRENT_FILTER_TIME                         128       /* [4ms] (Must be a power of 2, max 65536) Filter the robot current sensors for this long. (This many samples x2 sensors) - Worst-case error from BUS+filter delay with filtertime 128 is: (8ms + 4ms) * 100 / (128*4ms) = 2.4%. And 2.4% of 30A is 0.72A. */
#define MASTERBOARD_MAX_24V_CURRENT                           (float) 1.4       /* [A] (FP) The current where an IO overcurrent is generated. The specified max is lower */
#define MASTERBOARD_USING_INTERNAL_24V_PSU_THRESSHOLD         (float) 9.5       /* [V] (FP) If the I/O voltage is lower than this thresshold voltage at powerup then enable the internal 24V power supply */
#define MASTERBOARD_5V_REGULATOR_VOLTAGE_MAX                  (float) 5.4       /* [V] (FP) Max. allowed voltage from 5V regulator output. - Remember that there are tolerances on the ADC measurement too */
#define MASTERBOARD_5V_REGULATOR_VOLTAGE_MIN                  (float) 4.6       /* [V] (FP) Min. allowed voltage from 5V regulator output. */
#define MASTERBOARD_12V_INPUT_VOLTAGE_MAX                     (float) 12.5      /* [V] (FP) Max. allowed voltage from 12V input. - Remember that there are tolerances on the ADC measurement too */
#define MASTERBOARD_12V_INPUT_VOLTAGE_MIN                     (float) 10.2      /* [V] (FP) Min. allowed voltage from 12V input. */
#define MASTERBOARD_M12V_REGULATOR_VOLTAGE_MAX                (float) -9.0      /* [V] (FP) Max. allowed voltage from -12V regulator. - Remember that there are tolerances on the ADC measurement too */
#define MASTERBOARD_M12V_REGULATOR_VOLTAGE_MIN                (float) -13.0     /* [V] (FP) Min. allowed voltage from -12V regulator. */
#define MASTERBOARD_ROBOT_SHORT_CIRCUIT_THRESHOLD             (float) 7.0       /* [V] (FP) The robot voltage must be higher than this value when softstart is turned on else the robot must be short circuited */
#define MASTERBOARD_MAX_ROBOT_VOLTAGE_BEFORE_POWERUP          (float) 4.0       /* [V] (FP) Max. voltage on 48V robot power BUS at boot and before powering up the robot. Shall be low enough to ensure that the uP in the robot are without power. Shall also be low enough to ensure that the robot current is ~0mA, else the current offset calibration will be calibrated badly) */
#define MASTERBOARD_48V_VOLTAGE_TOLERANCE_AT_UPSTART          (float) 2.0       /* [V] (FP) Max. allowed voltage tolerance on the 48V voltage PSU when the robot is powered up */
#define MASTERBOARD_MIN_48V_VOLTAGE                           (float) 30.0      /* [V] (FP) Min. allowed 48V voltage when the robot is working (The masterboard just need voltage enough to generate 24V and therefor set it til at conservative value */
#define MASTERBOARD_MAX_48V_VOLTAGE                           (float) 58.0      /* [V] (FP) Max. allowed 48V voltage when the robot is working (This value is only exceeded if the energy eater is broken) */
#define MASTERBOARD_MAX_ROBOT_VOLTAGE_WHEN_UNPOWERED          (float) 18.0      /* [V] (FP) Max. allowed robot voltage when the robot is off. This voltage shall be low enouth to ensure a safe check of the power mosfets contrlled by the secondary processor and the voltage shall be high enough to avoid a fault while the engineers are backdriving a robot without poser. This value shall always be lower than MASTERBOARD_MIN_48V_VOLTAGE. The robot normally breakes from software so that the voltage on an unpowered robot is not higher than 11V*/
#define MASTERBOARD_ROBOT_CURRENT_OFFSET_MAX_TOLERANCE        (float) 3.0       /* [A] (FP) Max. allowed offset current error on robot current sensor ACS712 before calibration */
#define MASTERBOARD_MAX_ROBOT_CURRENT_DIFFERENS               (float) 1.0       /* [A] (FP) Max. differens between the two robot current sensors. If the value is exceeded a sensor must be broken */
#define MASTERBOARD_MAX_ROBOT_CURRENT_IN_ONE_SENSOR           (float) 15.0      /* [A] (FP) Used for fast short circuit protection. Note that there is no filter on this reading. This value shall be higher for at big robot. */
#define MASTERBOARD_MAX_SAFEGUARD_ROBOT_CURRENT               (float) 3.3       /* [A] (FP) KIEFFER: Koncept? 80W?? robot current in safeguard stop state (after stop time) */
#define MASTERBOARD_MAX_EMERGENCY_ROBOT_CURRENT               (float) 0.8       /* [A] (FP) Max. robot current in emergency stop state (after stop time) */
#define MASTERBOARD_MAIN_POWER_OFF_THRESSHOLD_CURRENT         (float) 0.35      /* [A] (FP) When the 12V PC current is lower than this value the masterboard will turn off all the power (after timeout) */
#define MASTERBOARD_MAX_TEMP                                  (float) 80.0      /* [Degrees celcius]     Max temp inside controller box. Generate fault when higher. */
#define MASTERBOARD_MAX_SAFETY_STATE_CHANGES_WITHIN_LIFETIME          5         /* [] (32bit) A fault is generated if the number is exceded. This functionality prevents divergence of safety state which might create a dangerous situation */
#define MASTERBOARD_MAX_ROBOT_ON_OFF_CHANGES_WITHIN_LIFETIME          6         /* [] (32bit)  Same consept as with the safety state  */
#define MASTERBOARD_REPORT_ERROR_BUFFER_SIZE                          64        /* [2x bytes] (Must be a power of 2, max 128) The buffer size for report_error codes */
#define EUROMAP67_MAX_24V_CURRENT                             (float) 2.2       /* [A] (FP) The current where an IO overcurrent is generated. The specified max is lower */
#define EUROMAP67_5V_VOLTAGE_MAX                              (float) 5.4       /* [V] (FP) Max. allowed voltage from 5V-bus. - Se note in diagram regarding the voltage drop on the GND when the 48V powers up + soft start on 24V regulators. Note: if the voltage drops below 4V the 3v3 voltage will drop, and the ADC value from meassuring the 5V might raise again */
#define EUROMAP67_5V_VOLTAGE_MIN                              (float) 4.1       /* [V] (FP) Min. allowed voltage from 5V-bus. 5V is not critical because the RS485 transcievers will work at 3v3 too*/
#define EUROMAP67_12V_VOLTAGE_MAX                             (float) 12.5      /* [V] (FP) Max. allowed voltage from 12V input. - Remember that there are tolerances on the ADC measurement too */
#define EUROMAP67_12V_VOLTAGE_MIN                             (float)  9.5      /* [V] (FP) Min. allowed voltage from 12V input. */
#define EUROMAP67_MIN_48V_VOLTAGE                             (float) 26.0      /* [V] (FP) Min. allowed 48V voltage. The 24V voltage will probably drop a few volt with 26V at the input but the main check is if the ferrits/fuses are blown or not (hmm... maybe 26V is too low when the regulator is running in DCM)*/



/*********************************************************************************************************
* System specific definitions (Should not be changed)
*********************************************************************************************************/
#define BAUD_RATE_RS485 (1875000/2) /*  bps */
#define UR_ETHERNET_DEVICENAME "/dev/urobot"
#define ROBOT_CONTROLLER_FREQUENCY 125 /*  Hz  */
#define NUMBER_OF_JOINTS 6
#define NUMBER_OF_TOOL_BOARDS 1

#define PC_INBUFFER_SIZE 255      /* [Bytes] (max. 255) */
#define PC_OUTBUFFER_SIZE 255     /* [Bytes] (max. 255) */
#define JOINT_INBUFFER_SIZE 255   /* [Bytes] (max. 255) (The same number of bytes is used for a TOOL) */
#define JOINT_OUTBUFFER_SIZE 255  /* [Bytes] (max. 255) */

#ifndef TRUE
#define TRUE (2==2)
#define FALSE (2==3)
#endif

#ifndef int16_t
#include <stdint.h>
#endif

#ifndef int8
#define int8 char
#define uint8 unsigned char
#define int16 short
#define uint16 unsigned short
#define int32 long
#define uint32 unsigned long
#endif

#define IO_DIGITAL_INPUT_PORTS 10
#define IO_DIGITAL_OUTPUT_PORTS 10
#define IO_ANALOG_INPUT_PORTS 4
#define IO_ANALOG_OUTPUT_PORTS 2
#define NUMBER_OF_EUROMAP_INPUT_BITS 21
#define NUMBER_OF_EUROMAP_OUTPUT_BITS 16

/*********************************************************************************************************
* Type Definitions
*********************************************************************************************************/
#define NODE_TYPE_SCREEN_PROCESSOR_1      65   /* Node Type is the same ID where bit 64 when the ID is placed on the SAFETY BUS */
#define NODE_TYPE_SCREEN_PROCESSOR_2      66
#define NODE_TYPE_EUROMAP67_PROCESSOR_1   67
#define NODE_TYPE_EUROMAP67_PROCESSOR_2   68
#define NODE_TYPE_SECONDARY_PROCESSOR     69 

struct CalibrationValues {
  int16_t flash_write_count;
  int16_t joint_id;
  uint16_t motor_encoder_commutation_offset; /* In ticks */
  uint16_t revision; /* Was spare, FFFF=Before ControllerBox2.0, 0001=Version 1.3 software, indicates partD */
  int32_t max_gear_stroke; /* In APOS_U */
  int32_t motor_encoder_to_joint_factory_calibration_value; /* In APOS_U */
  int32_t joint_encoder_factory_calibration_value; /* In APOS_U */

  uint16_t AD_Voltage_offset_calibration; /* Actually not used	 */
  uint16_t AD_Motor_temp_offset_calibration; 	 /* Actually not used */	
  uint16_t AD_Electronics_temp_offset_calibration; 	 /* Actually not used */	
  uint8_t joint_hardware_flags; /* [xxxxxxdd], dd=joint_encoder_direction, 0=undef, 1=pos, 2=neg */
  uint8_t AD_Accelerometer_x_offset_calibration; 	 /* Actually not used */	
  uint16_t AD_Accelerometer_xy_offset_calibration;  /* Actually not used */		
  uint16_t AD_Accelerometer_yz_offset_calibration; 	 /* Actually not used */	 
  uint16_t checksum_part_A;
  
  uint16_t AD_HallA_offset_calibration; /* These are set at every startup */
  uint16_t AD_HallB_offset_calibration; /* These are set at every startup */
  uint16_t checksum_part_B;
  
  int32_t joint_revolution_counter;
  uint16_t last_seen_joint_position;
  uint16_t checksum_part_C;
  
  int16_t data[100]; /* Indices 0-12 are used by part d calibration */
  uint16_t checksum_part_D; /* Total 250 bytes */
};

/* The state of safety-related uPs (NOTE! The priority is falling) */
enum SAFETY_STATE {                       /* Safety Related States for statemachines on the two uPs on the masterboard */
  SAFETY_STATE_UNDEFINED,                 /* Used for shared state variables when communication is down */
  SAFETY_STATE_BOOTLOADER,                /* Used for bootloading firmware over the robot bus */
  SAFETY_STATE_FAULT,                     /* Used for internal circuit/robot errors only */
  SAFETY_STATE_BOOTING,                   /* Waiting for communication to work, robot voltage to be under 4V. This state is also used to reboot the masterboard instead of faulting. */
  SAFETY_STATE_INITIALIZING,              /* Initializing masterboard (checking voltages etc.) */
  SAFETY_STATE_ROBOT_EMERGENCY_STOP,      /* (EA + EB + SBUS->Screen)    Physical e-stop interface input activated */
  SAFETY_STATE_EXTERNAL_EMERGENCY_STOP,   /* (EA + EB + SBUS->Euromap67) Physical e-stop interface input activated */
  SAFETY_STATE_SAFEGUARD_STOP,            /* (SA + SB + SBUS) Physical   s-stop interface input activated */
  SAFETY_STATE_OK                         /* The robot is active and happy */
};

/* Robot ON/OFF state */
enum ROBOT_48V_STATE {
  ROBOT_48V_STATE_OFF,                    /* There is ~0V voltage on the robot */
  ROBOT_48V_STATE_TURNING_ON,             /* Voltage is raising (Soft Start) */
  ROBOT_48V_STATE_ON,                     /* Power transistors is on, and the voltage is ~48V */
  ROBOT_48V_STATE_TURNING_OFF             /* Voltge is falling towards ~0V */
};

enum ELECTRONICS_TYPE {
  ELECTRONICS_TYPE_UNDEFINED,
  ELECTRONICS_TYPE_PRIMARY_SCREEN,
  ELECTRONICS_TYPE_SECONDARY_SCREEN,
  ELECTRONICS_TYPE_PRIMARY_EUROMAP67,
  ELECTRONICS_TYPE_SECONDARY_EUROMAP67,
  ELECTRONICS_TYPE_SECONDARY_MASTERBOARD
};


/* Safety Information (the safety information package is different between primary and secondary uP's on the masterboard) */
#define SAFETY_INFORMATION_RESET_ACTIVE             (1<<0)  /* (uP1 -> uP2) The bit is 1 if the filtered Safeguard Reset input is active */
#define SAFETY_INFORMATION_AUTO_ACTIVE              (1<<0)  /* (uP2 -> uP1) The bit is 1 if the filtered Safeguard Auto input is active */
#define SAFETY_INFORMATION_EUROMAP67_ACTIVATED      (1<<1)  /* (uP2 -> uP1) The bit is 1 if a Euromap67 interface is activated (stored in data flash of the secondary uP) */
#define SAFETY_INFORMATION_REMOVE_EUROMAP67         (1<<1)  /* (uP1 -> uP2) The bit is 1 if a stored "Euromap67 interface active" bit should be deleted (stored in data flash of the secondary uP) */
#define SAFETY_INFORMATION_STAY_IN_SAFEGUARD_STOP   (1<<2)  /* (uP1 -> uP2) The bit is 1 if the secondary masterboard uP shall stay in Safeguard Stop and not go to OK safety state (This hit is high if the primary (ColdFire) uP is waiting for a reset-butten-press */
#define SAFETY_INFORMATION_TEACH_PENDANT_ACTIVATED  (1<<3)  /* (uP2 -> uP1) The bit is 1 if the teach pendant safety is activated (stored in data flash of the secondary uP) */
#define SAFETY_INFORMATION_REMOVE_TEACH_PENDANT     (1<<3)  /* (uP1 -> uP2) The bit is 1 if a stored "Screen interface active" bit should be deleted (stored in data flash of the secondary uP) */

#define SCREEN_INFORMATION_MENU                     (1<<0)  /* Menu button */
#define SCREEN_INFORMATION_SELECT                   (1<<1)  /* Select button */
#define SCREEN_INFORMATION_DOWN                     (1<<2)  /* Down button */
#define SCREEN_INFORMATION_UP                       (1<<3)  /* Up button */


#endif /* __MICROPROCESSOR_UTILITIES_H_ */
