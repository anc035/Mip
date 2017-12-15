/*******************************************************************************
* balance.c
*
* Balance code for BeagleBone Blue and EduMip
* 
*******************************************************************************/

// main roboticscape API header
#include <roboticscape.h>
#include <rc_usefulincludes.h>
#include "balance_config.h"

// function declarations
void on_pause_pressed();
void on_pause_released();

/*******************************************************************************
* control_state_t
* ENGAGED or DISENGAGED to show if controller is running
*
*******************************************************************************/
typedef enum control_state_t{
	ENGAGED,
	DISENGAGED
}control_state_t;

/*******************************************************************************
* setpoint_t
* 
* Controller setpoint written by setpoint_mgr and read by controller
*******************************************************************************/
typedef struct setpoint_t{
	control_state_t control_state;
	float theta;		//body theta radians
	float phi;		// wheel position radians
	float phi_dot;		//rate at which phi ref updates radians/s
	float gamma;		//body turn angle radians
	float gamma_dot;	// rate of gamma setpoint updates radians/s
}setpoint_t;
/*******************************************************************************
* sys_state_t
* System information
*
*******************************************************************************/
typedef struct core_state_t{
	float wheelAngleL; //wheel angle
	float wheelAngleR;
	float theta;	   //Mip angle radians
	float phi;	   //average wheels angle
	float gamma;	   //turn angle radians 
	float vBatt;	   // battery status
	float d1_out;	   //output to motors
	float d2_out;	   //theta_ref
	float d3_out;	   //steering output
	float mot_drive;   //output correction for battery voltage	
} core_state_t;

/*******************************************************************************
* Local Functions  
*
*
*******************************************************************************/
//IMU interrupt service
void balancer();
//threads
void* printer(void* ptr);
void* battery_checker(void* ptr);
void* setpoint_manager(void*ptr);
//functions
int zero_out_controller();
int wait_for_start_condition();
int disengage_controller();
int engage_controller();

/*******************************************************************************
* Global Variables 
*
*
*******************************************************************************/
core_state_t state;
setpoint_t setpoint;
rc_imu_data_t imu_data;
rc_ringbuf_t accel_in_buf;
rc_ringbuf_t accel_out_buf;
rc_ringbuf_t gyro_in_buf;
rc_ringbuf_t gyro_out_buf;
rc_ringbuf_t d1_in_buf;
rc_ringbuf_t d1_out_buf;

/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){

	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}
	// do your own initialization here
	printf("\nHello BeagleBone\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	rc_set_state(UNINITIALIZED);

	setpoint.control_state = DISENGAGED;
	//setup ring bufs
  accel_in_buf=rc_empty_ringbuf();
  accel_out_buf=rc_empty_ringbuf();
  gyro_in_buf=rc_empty_ringbuf();
  gyro_out_buf=rc_empty_ringbuf();

  if(rc_alloc_ringbuf(&accel_in_buf,5)<0){
          printf("accel in ringbuf allocation failed\n");
  }
  if(rc_alloc_ringbuf(&accel_out_buf,5)<0){
          printf("accel out ringbuf allocation failed\n");
  }
  if(rc_alloc_ringbuf(&gyro_in_buf,5)<0){
          printf("gyro in ringbuf allocation failed\n");
  }
  if(rc_alloc_ringbuf(&gyro_out_buf,5)<0){
          printf("gyro out ringbuf allocation failed\n");
  }
	
	//sample battery thread
	pthread_t battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	//wait for battery thread to make first read
	while(state.vBatt==0 && rc_get_state()!=EXITING) rc_usleep(1000);

	
	//start printer if running from terminal
	pthread_t print_thread;
	if(isatty(fileno(stdout))){
		pthread_create(&print_thread,NULL,printer,(void*) NULL);
	}

	//set up IMU configuration
	rc_imu_config_t imu_config= rc_default_imu_config();
	imu_config.dmp_sample_rate=SAMPLE_RATE_D1_HZ;
	
	//start imu
	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: IMU failed to initialize\n");
		rc_blink_led(RED,5,5);
		return -1;
	}
	// start setpoint thread
	pthread_t setpoint_thread;
	pthread_create(&setpoint_thread, NULL, setpoint_manager, (void*) NULL);

	//Interrupt set last
	rc_set_imu_interrupt_func(&balancer);
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	printf("\nHold your MIP upright to begin balancing\n");

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// always sleep at some point
		rc_usleep(10000);
	}
	
	// exit cleanly
	rc_power_off_imu();
	rc_cleanup(); 
	if(pthread_join(print_thread,NULL)==0){
		printf("\nprint thread joined\n");
	}
	return 0;
}

/*******************************************************************************
* void setpoint_manager(void* ptr
*	
* Adjusts controller setpoint if radio control is used. Detects control engaging
* called at SAMPLE_RATE_HZ (See configuration file)
*******************************************************************************/
void* setpoint_manager(void* ptr){
		//wait for IMU to settle
		disengage_controller();
		rc_usleep(2500000);
		rc_set_state(RUNNING);
		rc_set_led(RED,0);
		rc_set_led(GREEN,1);

		while(rc_get_state()!=EXITING){
			// sleep at beginning to use continue statement
			rc_usleep(1000000/SETPOINT_MANAGER_HZ);

			//do nothing if paused
			if(rc_get_state() !=RUNNING) continue;

			//if control disengaged wait for start condition
			if(setpoint.control_state ==DISENGAGED){
				if(wait_for_start_condition()==0){
					zero_out_controller();
					engage_controller();
				}
				else 
				setpoint.theta=0;
				setpoint.phi_dot=0;
				setpoint.gamma_dot =0;
				continue;
			}
		}
		disengage_controller();
		return NULL;
}



/*******************************************************************************
* void balancer()          
*	
* discrete-time balance controller using IMU interrupt function
* called at SAMPLE_RATE_HZ (See configuration file)
*******************************************************************************/
void balancer(){
	static int inner_saturation_counter =0;
	float dutyL,dutyR;
	/*****************************************************************
	*Complementary filter LPF for Accelerometer and HPF for Gyroscope
	*
	*****************************************************************/
	// calculate accel angle of Z over Y
  float theta_a_raw=atan2(-imu_data.accel[2],imu_data.accel[1]);
  // Euler integration of gyro data X 
  static float theta_g_raw=0;
	theta_g_raw=theta_g_raw+imu_data.gyro[0]*DT_D1*DEG_TO_RAD;

	//obtain encoder positions and convert to wheel angles
	state.wheelAngleR=(rc_get_encoder_pos(ENCODER_CHANNEL_R)*TWO_PI )\
				/(ENCODER_POLARITY_R * GEARBOX *ENCODER_RES);
       	state.wheelAngleL=(rc_get_encoder_pos(ENCODER_CHANNEL_L)*TWO_PI )\
				/(ENCODER_POLARITY_L * GEARBOX *ENCODER_RES);
	//input body angle estimate into ringbuffers
	rc_insert_new_ringbuf_value(&accel_in_buf,theta_a_raw);
  rc_insert_new_ringbuf_value(&gyro_in_buf,theta_g_raw);
  float accel_in_old=rc_get_ringbuf_value(&accel_in_buf,1);
  float accel_out_old=rc_get_ringbuf_value(&accel_out_buf,1);
  float gyro_in_old=rc_get_ringbuf_value(&gyro_in_buf,1);
  float gyro_out_old=rc_get_ringbuf_value(&gyro_out_buf,1);

  //LPF for accelerometer tf=(w*h)/(z+(w*h-1))
  float theta_a=FILTER_W*DT_D1*(accel_in_old-accel_out_old)+accel_out_old;
  //HPF for gyroscope tf= (z-1)/(z+(w*h-1))
  float theta_g=theta_g_raw-gyro_in_old-(FILTER_W*DT_D1-1.0)*gyro_out_old;
  //update buffer
  rc_insert_new_ringbuf_value(&accel_out_buf,theta_a);
  rc_insert_new_ringbuf_value(&gyro_out_buf,theta_g);

	//body theta estimate with an offset included
  state.theta=theta_a+theta_g+MOUNT_ANGLE;
	//average wheel rotation with body rotation 
	state.phi=((state.wheelAngleL+state.wheelAngleR)/2)+state.theta;
	//steering angle 
	state.gamma =(state.wheelAngleR-state.wheelAngleL) \
					*(WHEEL_RADIUS_M/TRACK_WIDTH_M);

	/*******************************************************
	*check for exit conditions after state estimation
	*******************************************************/
	if(rc_get_state()==EXITING){
		rc_disable_motors();
		return;
	}
	// if controller ENGAGED while state is PAUSED, DISENGAGE
	if(rc_get_state()!=RUNNING && setpoint.control_state==ENGAGED){
		disengage_controller();
		return;
	}
	// exit if the controller is disengaged
	if(setpoint.control_state==DISENGAGED){
		return;
	}

	//check for a tipover
	if(fabs(state.theta)>TIP_ANGLE){
		disengage_controller();
		printf("tip detected \n");
		return;
	}

/*******************************************************************************
 * INNER LOOP ANGLE Theta controller D1
 * Input to D1 is theta error(setpoint-state). Then scale output u to compensate
 * for changing battery voltage.
*******************************************************************************/
	float d1_gain=D1_GAIN * V_NOMINAL/state.vBatt;
		

	return;
}

/*******************************************************************************
* zero_out_controller() 
*	
* Clear the controller's memory and zero out setpoints. 
*******************************************************************************/
int zero_out_controller(){
	rc_reset_ringbuf(&accel_in_buf);
	rc_reset_ringbuf(&accel_out_buf);
	rc_reset_ringbuf(&gyro_in_buf);
	rc_reset_ringbuf(&gyro_out_buf);
	setpoint.theta =0.0f;
	setpoint.phi   =0.0f;
	setpoint.gamma =0.0f;
	rc_set_motor_all(0.0f);
	return 0;
}

/*******************************************************************************
* disengage_controller()
*
* disable motors & set the setpoint.core_mode to DISENGAGED
*******************************************************************************/
int disengage_controller(){
	rc_disable_motors();
	setpoint.control_state = DISENGAGED;
	return 0;
}

/*******************************************************************************
* engage_controller()
*
* zero out the controller & encoders. Enable motors & arm the controller.
*******************************************************************************/
int engage_controller(){
	zero_out_controller();
	rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
	rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
	setpoint.control_state = ENGAGED;
	rc_enable_motors();
	return 0;
}

/*******************************************************************************
* int wait_for_start_condition()
*
* Waits for Mip to be held upright long enough to begin
*Returns 0 if successful. Returs -1 if wait was interrupted by pause or shutdown
*******************************************************************************/
int wait_for_start_condition(){
	int checks=0;
	const int check_hz=20;   //check 20 times per second
	int checks_needed=round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz;

	//wait for Mip to tip forward or back 
	//exit if state set to paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(state.theta) > START_ANGLE) checks++;
		else checks=0;
		// return after waiting too long
		if(checks>= checks_needed) break;
		rc_usleep(wait_us);
	}
	//wait for Mip to be upright 
	checks =0;
	// exit if state is set to paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range,start counting
		if(fabs(state.theta)<START_ANGLE) checks++;
		//falls out of range, restart counter
		else checks=0;
		//waited long enough then return
		if(checks >= checks_needed) return 0;
		rc_usleep(wait_us);
	}
	return -1;
}

/*******************************************************************************
* printer
*
* prints status to console only started if executing from terminal.
*******************************************************************************/
void* printer(void* ptr){
	rc_state_t last_rc_state, new_rc_state; //keeping track of previous state
	last_rc_state=rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state=rc_get_state();

		// check if first time being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("    γ    |");
			printf("  D1_u   |");
			printf("  D3_u   |");
			printf("  vBatt  |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;
		// decide what to print or exit
		if(new_rc_state == RUNNING){	
			printf("\r");
			printf("%7.3f  |", state.theta);
			printf("%7.3f  |", setpoint.theta);
			printf("%7.3f  |", state.phi);
			printf("%7.3f  |", setpoint.phi);
			printf("%7.3f  |", state.gamma);
			printf("%7.3f  |", state.d1_out);
			printf("%7.3f  |", state.d3_out);
			printf("%7.3f  |", state.vBatt);
			
			if(setpoint.control_state == ENGAGED) printf("  ENGAGED  |");
			else printf("DISENGAGED |");
			
		}
			fflush(stdout);
		
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}		

/*******************************************************************************
 * battery_checker()
 *
 * Slow loop checking battery voltage and change D1 saturation limit 
*******************************************************************************/
void* battery_checker(void* ptr){
	float new_v;
	while(rc_get_state()!=EXITING){
			new_v= rc_battery_voltage();
			// if over range of battery set to Vnominal
			if(new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
			state.vBatt = new_v;
			rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
