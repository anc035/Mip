/*******************************************************************************
* balance.c
*
* Balance code for BeagleBone Blue and EduMip
* Includes a D1 inner loop for balancing, a D2 outer loop for keeping Mip near 
* setpoint, a D3 steering controller to correct any turning caused by the first 
* two controllers.
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
* stores setpoints
*******************************************************************************/
typedef struct setpoint_t{
	control_state_t control_state;
	float theta;		//body theta radians
	float phi;		// wheel position radians
	float gamma;		//body turn angle radians
}setpoint_t;
/*******************************************************************************
* core_state_t 
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
void* outer_loop(void* ptr);
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
rc_ringbuf_t d1_in_buf;
rc_ringbuf_t d1_out_buf;
rc_ringbuf_t d2_in_buf;
rc_ringbuf_t d2_out_buf;
rc_ringbuf_t d3_in_buf;
rc_ringbuf_t d3_out_buf;

static float soft_start=0;
float theta_a=0.0;
float theta_g=0.0; //ale



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

	// start with Disengaged state to detect when Mip is picked up
	setpoint.control_state = DISENGAGED;
	//empty  ring bufs 
	d1_in_buf    =rc_empty_ringbuf();
	d1_out_buf   =rc_empty_ringbuf();
	d2_in_buf    =rc_empty_ringbuf();
	d2_out_buf   =rc_empty_ringbuf();
	d3_in_buf    =rc_empty_ringbuf();
	d3_out_buf   =rc_empty_ringbuf();
	
  
	if(rc_alloc_ringbuf(&d1_in_buf,4)<0){
          printf("d1 in ringbuf allocation failed\n");
	}
	if(rc_alloc_ringbuf(&d1_out_buf,4)<0){
          printf("d1 out ringbuf allocation failed\n");
	}
	if(rc_alloc_ringbuf(&d2_in_buf,4)<0){
          printf("d2 in ringbuf allocation failed\n");
	}
	if(rc_alloc_ringbuf(&d2_out_buf,4)<0){
          printf("d2 out ringbuf allocation failed\n");
	}	
	if(rc_alloc_ringbuf(&d3_in_buf,4)<0){
          printf("d3 in ringbuf allocation failed\n");
	}
	if(rc_alloc_ringbuf(&d3_out_buf,4)<0){
          printf("d3 out ringbuf allocation failed\n");
	}

		//outer loop thread
	pthread_t d2_thread;
	pthread_create(&d2_thread,NULL,outer_loop, (void*) NULL);
	pthread_setschedprio(d2_thread,60);
	//sample battery thread
	pthread_t battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	pthread_setschedprio(battery_thread, 22);
	//wait for battery thread to make first read
	while(state.vBatt==0 && rc_get_state()!=EXITING) rc_usleep(1000);

	//printer thread to print to screen 	
	pthread_t print_thread;
	pthread_create(&print_thread,NULL,printer,(void*) NULL);
	pthread_setschedprio(print_thread,25);
	

	//set up IMU configuration
	rc_imu_config_t imu_config= rc_default_imu_config();
	imu_config.dmp_sample_rate=SAMPLE_RATE_D1_HZ;
	imu_config.dmp_interrupt_priority=99;
	
	//start imu
	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: IMU failed to initialize\n");
		rc_blink_led(RED,5,5);
		return -1;
	}

	//Interrupt set last
	rc_set_imu_interrupt_func(&balancer);


	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	printf("\nHold your MIP upright to begin balancing\n");

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		//detect starting condition(when Mip is picked up
		if(setpoint.control_state ==DISENGAGED){
				if(wait_for_start_condition()==0){
					engage_controller();
					rc_set_led(RED,0);
					rc_set_led(GREEN,1);
					}
			}
		// always sleep at some point
		rc_usleep(10000);
	}
	
	// exit cleanly
	rc_power_off_imu();
	rc_cleanup(); 
	rc_disable_motors();
	if(pthread_join(print_thread,NULL)==0){
		printf("\nprint thread joined\n");
	}
	if(pthread_join(battery_thread,NULL)==0){
		printf("\nbattery thread joined\n");
	}
	if(pthread_join(d2_thread,NULL)==0){
		printf("\nd2_thread joined\n");
	}

	return 0;
}



/*******************************************************************************
* void balancer()          
*	
* discrete-time balance controller using IMU interrupt function
* called at SAMPLE_RATE_HZ (See configuration file)
*******************************************************************************/
void balancer(){
	//initializing variables
	static int inner_saturation_counter =0;
	float dutyL,dutyR;
	float	d1_num[]=D1_NUM;
	float	d1_den[]=D1_DEN;
	float d3_num[]=D3_NUM;
	float d3_den[]=D3_DEN;

	
	/*****************************************************************
	*Complementary filter LPF for Accelerometer and HPF for Gyroscope
	* Used for state estimation
	*****************************************************************/
	//initialize values to zero	
	float theta_a_raw=0;
	static float theta_g_raw = 0;
	static float last_theta_a_raw = 0;
	static float last_theta_g_raw = 0;
	static float last_theta_a = 0;
	static float last_theta_g = 0;
	//calculate angle from acceleration data
	theta_a_raw = atan2(-imu_data.accel[2],imu_data.accel[1]);
	//calculate rotation from start with gyro data
	theta_g_raw = theta_g_raw + DT_D1*(imu_data.gyro[0]*DEG_TO_RAD) ;
	// Low Pass Filter for accelerometer
	theta_a = (FILTER_W*DT_D1*last_theta_a_raw)+((1-(FILTER_W*DT_D1))*last_theta_a);
	// High pass filter for gyroscope
	theta_g = (1-(FILTER_W*DT_D1))*last_theta_g + theta_g_raw - last_theta_g_raw;
	//get theta 
	state.theta = theta_a + theta_g + MOUNT_ANGLE;

	//set last stuff
	last_theta_a = theta_a;
	last_theta_g = theta_g;
	last_theta_g_raw = theta_g_raw;
	last_theta_a_raw = theta_a_raw;

		//steering angle  calculation
	state.wheelAngleR= (rc_get_encoder_pos(ENCODER_CHANNEL_R) *TWO_PI)\
												 /(ENCODER_POLARITY_R *GEARBOX *ENCODER_RES);
	state.wheelAngleL= (rc_get_encoder_pos(ENCODER_CHANNEL_L) *TWO_PI)\
												 /(ENCODER_POLARITY_L *GEARBOX *ENCODER_RES);

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
		printf("\ntip detected state.theta %f\n",state.theta);
		return;
	}

/*******************************************************************************
 * INNER LOOP ANGLE Theta controller D1
 * Input to D1 is theta error(setpoint-state). Then scale output u to compensate
 * for changing battery voltage.
*******************************************************************************/
	rc_insert_new_ringbuf_value(&d1_in_buf,setpoint.theta-state.theta);
	
	state.d1_out=soft_start*D1_GAIN*(d1_num[0]*rc_get_ringbuf_value(&d1_in_buf,0) \
													  +(d1_num[1]*rc_get_ringbuf_value(&d1_in_buf,1)) \
														+(d1_num[2]*rc_get_ringbuf_value(&d1_in_buf,2))\
							 							-(d1_den[1]*rc_get_ringbuf_value(&d1_out_buf,0))
														-(d1_den[2]*rc_get_ringbuf_value(&d1_out_buf,1)));
	rc_insert_new_ringbuf_value(&d1_out_buf,state.d1_out);
	
/*******************************************************************************
*Inner loop saturation check if saturated over a second disable controller
*
*******************************************************************************/
	if(fabs(state.d1_out)>0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0;
	//if saturate for a second disable
	if(inner_saturation_counter > (SAMPLE_RATE_D1_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated \n");
		disengage_controller();
		inner_saturation_counter = 0;
		return;
	}
	if(soft_start<1)soft_start+=.1;
	if(soft_start>=1)soft_start=1;

/*******************************************************************************
 * D3 controller for gamma changes
 * 
*******************************************************************************/

	rc_insert_new_ringbuf_value(&d3_in_buf,setpoint.gamma-state.gamma);
	state.d3_out=D3_GAIN*((d3_num[0]*rc_get_ringbuf_value(&d3_in_buf,0)) \
											+(d3_num[1]*rc_get_ringbuf_value(&d3_in_buf,1)) \
											-(d3_den[1]*rc_get_ringbuf_value(&d3_out_buf,0)));
	rc_insert_new_ringbuf_value(&d3_out_buf,state.d3_out);
	//if the output of D3 is over  a value set it equal to that value
	if(fabs(state.d3_out) >STEERING_INPUT_MAX) state.d3_out=STEERING_INPUT_MAX;
	if(fabs(state.d3_out)<-STEERING_INPUT_MAX) state.d3_out=-STEERING_INPUT_MAX;

/*******************************************************************************
 * Send signal to motors
 * add D1 balance control u and D3 steering control
 *multiplied by polarity to enure direction
*******************************************************************************/

	dutyL =state.d1_out-state.d3_out;
	dutyR =state.d1_out+state.d3_out;
	rc_set_motor(MOTOR_CHANNEL_L,MOTOR_POLARITY_L * dutyL);
	rc_set_motor(MOTOR_CHANNEL_R,MOTOR_POLARITY_R * dutyR);

	return;
}
	
/*******************************************************************************
* zero_out_controller() 
*	
* Clear the controller's memory and zero out setpoints. 
*******************************************************************************/
int zero_out_controller(){
	rc_reset_ringbuf(&d1_in_buf);
	rc_reset_ringbuf(&d1_out_buf);
	rc_reset_ringbuf(&d2_in_buf);
	rc_reset_ringbuf(&d2_out_buf);
	rc_reset_ringbuf(&d3_out_buf);
	rc_reset_ringbuf(&d3_out_buf);
	
	setpoint.theta =0.0f;
	setpoint.phi   =0.0f;
	setpoint.gamma =0.0f;
	rc_set_motor_all(0.0f);
	return 0;
}

/*******************************************************************************
* disengage_controller()
*
* disable motors & set the control_state to DISENGAGED
*******************************************************************************/
int disengage_controller(){
	rc_disable_motors();
	setpoint.control_state = DISENGAGED;
	rc_set_led(RED,1);
	return 0;
}

/*******************************************************************************
* engage_controller()
*
* zero out the controller & encoders. Enable motors & engage  the controller.
*******************************************************************************/
int engage_controller(){
	soft_start=0;
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
	printf("wait for start condition failed");
	return -1;
}

/*******************************************************************************
* printer
*
* prints status to the screen
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
			printf("control_state|");
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
			
		if(setpoint.control_state == ENGAGED) {
				printf("  ENGAGED  |");
		}
		else printf("DISENGAGED |");
			
		}
			fflush(stdout);
		
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}		

/*******************************************************************************
 * Outer loop thread()
 * change theta setpoint based on phi
 * input to the controller is phi error(setpoint-state)
 *
*******************************************************************************/
void* outer_loop(void* ptr){
	float d2_num[]=D2_NUM;
	float d2_den[]=D2_DEN;

	while(rc_get_state()!=EXITING){
		if(rc_get_state()==RUNNING && setpoint.control_state==ENGAGED){
			
			//average wheel rotation with body rotation 
			state.phi=((state.wheelAngleL+state.wheelAngleR)/2)+state.theta;

			rc_insert_new_ringbuf_value(&d2_in_buf,setpoint.phi-state.phi);
			state.d2_out=D2_GAIN*(d2_num[0]*rc_get_ringbuf_value(&d2_in_buf,0)   \
													 +(d2_num[1]*rc_get_ringbuf_value(&d2_in_buf,1))  \
													 -(d2_den[1]*rc_get_ringbuf_value(&d2_out_buf,0)));
			rc_insert_new_ringbuf_value(&d2_out_buf,state.d2_out);	
			setpoint.theta=state.d2_out;
		if(state.d2_out >THETA_REF_MAX) state.d2_out=THETA_REF_MAX;
		if(state.d2_out <-THETA_REF_MAX) state.d2_out=-THETA_REF_MAX;
		}
		rc_usleep(1000000 / SAMPLE_RATE_D2_HZ);
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
