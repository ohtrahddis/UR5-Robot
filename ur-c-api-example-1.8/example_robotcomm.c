#include <stdio.h>
#include "robotinterface.h"
#include "Configuration.h"
#include "microprocessor_commands.h"
#include "microprocessor_definitions.h"
#include <math.h>
#include <stdlib.h>

#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

/*
 * WARNING! WARNING! WARNING!
 * This program will automatically power on the robot and auto initialize it!
 * Make sure the robot will not collide with it self or anything else, 
 * before running this program!
 * WARNING! WARNING! WARNING!
 *
*/

char string[256];
int stringpointer = 0;
double position[6];
double readPosition[6];
double zero_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double position_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double speed_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double current_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

struct message_t message;
double orig_pos_1;

void print_joint_mode() {
  int joint;
  int mode;
  for (joint = 0; joint < 6; ++joint) {
    mode = robotinterface_get_joint_mode(joint);
    switch(mode) {
    case JOINT_MOTOR_INITIALISATION_MODE:
      printf("Joint %i mode: JOINT_MOTOR_INITIALISATION_MODE\n", joint);
      break;
    case JOINT_BOOTING_MODE:
      printf("Joint %i mode: JOINT_BOOTING_MODE\n", joint);
      break;
    case JOINT_POWER_OFF_MODE:
      printf("Joint %i mode: JOINT_POWER_OFF_MODE\n", joint);
      break;
    case JOINT_BOOTLOADER_MODE:
      printf("Joint %i mode: JOINT_BOOTLOADER_MODE\n", joint);
      break;
    case JOINT_CALIBRATION_MODE:
      printf("Joint %i mode: JOINT_CALIBRATION_MODE\n", joint);
      break;
    case JOINT_SECURITY_STOPPED_MODE:
      printf("Joint %i mode: JOINT_SECURITY_STOPPED_MODE\n", joint);
      break;
    case JOINT_FAULT_MODE:
      printf("Joint %i mode: JOINT_FAULT_MODE\n", joint);
      break;
    case JOINT_RUNNING_MODE:
      printf("Joint %i mode: JOINT_RUNNING_MODE\n", joint);
      break;
    case JOINT_INITIALISATION_MODE:
      printf("Joint %i mode: JOINT_INITIALISATION_MODE\n", joint);
      break;
    case JOINT_IDLE_MODE:
      printf("Joint %i mode: JOINT_IDLE_MODE\n", joint);
      break;
    default:
      printf("Joint %i mode: Unknown mode\n", joint);
    }
  }
}

int
main(int argc __attribute__((unused)), char *argv[] __attribute__((unused))) {

  int i, j;
  pid_t pid;
  struct sched_param sch_param;
  int retryCounter;
  int maxAttemptsToPowerup;

  printf("Example program start\n");
  message.text = string;
  
  printf("Loading robot configuration\n");
  configuration_load();

  printf("Setting RT priority\n");
  pid = getpid();
  sch_param.sched_priority = 99;
  if (sched_setscheduler(pid, SCHED_FIFO, &sch_param) == 0) {
    printf("- Prioority set\n");
  } else {
    printf("- Priority not set, error: %i\n", errno);
    exit(EXIT_FAILURE);
  }

  printf("Resetting positions\n");
  for (i = 0; i < 6; ++i) {
    position[i] = 0.0;
  }

  printf("Opeing robotinterface (real robot)\n");
  /* open robotinterface */
  if (robotinterface_open(0) == FALSE) {
    printf("robotinterface_open() failed\n");
    exit(EXIT_FAILURE);
  }

  retryCounter = 100;
  while (retryCounter>0 && !robotinterface_is_robot_connected()) {
    robotinterface_read_state_blocking();
    robotinterface_command_velocity(zero_vector);
    robotinterface_send();    
    retryCounter--;
  }
  
  if (!robotinterface_is_robot_connected()) {
    robotinterface_close();
    printf("Real robot not connected\n");
    exit(EXIT_FAILURE);
  }

  printf("Power up robot\n");
  maxAttemptsToPowerup = 5;
  for (i = 0; i < maxAttemptsToPowerup; ++i) {
    retryCounter=250;
    robotinterface_power_on_robot();
    while (retryCounter > 0) {
      robotinterface_read_state_blocking();
      robotinterface_command_empty_command();
      robotinterface_send();
      if (robotinterface_is_power_on_robot()) {
        break;
      }
      retryCounter--;
    }
  }
  
  if (!robotinterface_is_power_on_robot()) {
    robotinterface_close();
    printf("Unable to power up robot\n");
    exit(EXIT_FAILURE);
  }


  i = 0;
  printf("Initializing robot\n");
  do {
    ++i;
    robotinterface_read_state_blocking();
    if ((i & 255) == 0) { 
      printf("Current joint modes:\n");
      print_joint_mode(); 
      printf("\n");
    }
    for (j=0; j<6; ++j) {
      speed_vector[j] = (robotinterface_get_joint_mode(j) == JOINT_INITIALISATION_MODE) ? 0.1 : 0.0;
    }
    robotinterface_command_velocity(speed_vector);
    robotinterface_send();
  } while (robotinterface_get_robot_mode() == ROBOT_INITIALIZING_MODE);

  if (robotinterface_get_robot_mode() != ROBOT_RUNNING_MODE) {
    robotinterface_close();
    printf("Unable to initialize robot\n");
    exit(EXIT_FAILURE);
  }
  
  robotinterface_read_state_blocking();
  robotinterface_get_actual_position(position_vector);
  orig_pos_1 = position_vector[1];
  robotinterface_command_velocity(zero_vector);
  robotinterface_send();

  /* If we get here the robot is powered up and ready to rock and roll ! */

  printf("Running program\n");
  for (i = 0; (2 == 2); ++i) {

    robotinterface_read_state_blocking();

    /* Just make joint 1 move around */
    robotinterface_get_actual_position(position_vector);
    position_vector[1] = orig_pos_1 - 0.2 + 0.2 * cos(((double)i) / 1000.0);
    speed_vector[1] = -(0.2 / 1000.0) * sin(((double)i) / 1000.0);
    if ((i & 255) == 0) {
      printf("  %04x Setting pva command %f %f \n", i, position_vector[1], speed_vector[1]);
    }
    robotinterface_command_position_velocity_acceleration(position_vector, speed_vector, zero_vector);
    
    robotinterface_send();

    /* This could be done in a different thread. */
    while (robotinterface_get_message_count() > 0) {

      /* NOTE: it's not a good idea to print to the console inside the real time loop,
       * since this can ruin the realtime performace, leading to broken communication 
       * errors.  
       */

      /* printf("  %04x Handling message\n", i); */
      stringpointer = 0;
      stringpointer += robotinterface_get_message(&message);
      
      /* printf("%s \n", message.text); */
    }
  }
  printf("- Closing\n");
  robotinterface_close();
  printf("Done.\n");
  return 0;
}
