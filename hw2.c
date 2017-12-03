/*******************************************************************************
* rc_project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#define SAMPLE_RATE 100
#define TIME_CONSTANT 1.7
#define FILENAME "plot.txt"


// function declarations
void on_pause_pressed();
void on_pause_released();

/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	//new data struct
	rc_imu_data_t data;
	//initializing euler integration
	float theta_g_raw=0;
	const float dt=1.0/SAMPLE_RATE;
	const float w=1.0/TIME_CONSTANT;
	//file to store plotting data
	FILE *f;
	f=fopen(FILENAME, "w");
	if(f==0){
		perror(FILENAME);
		exit(1);
	}
	
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nHello BeagleBone\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	printf("\nSample Rate: %dhz\n",SAMPLE_RATE);
	printf("Time Constant: %5.2f\n",TIME_CONSTANT);

	//set imu to default config
	rc_imu_config_t conf=rc_default_imu_config();	

	if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	//setup ring buffers
	rc_ringbuf_t accel_in_buf=rc_empty_ringbuf();
	rc_ringbuf_t accel_out_buf=rc_empty_ringbuf();
	rc_ringbuf_t gyro_in_buf=rc_empty_ringbuf();
	rc_ringbuf_t gyro_out_buf=rc_empty_ringbuf();
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

	//print headers
	//printf(" Accel XYZ(m/s^2)   |");
	//printf(" Gyro XYZ (rad/s)   |");
	printf(" Theta a_raw (rads)|");
	printf(" g_raw (rads) |");
	printf(" Theta_a  |");
	printf(" Theta_g  |");
	printf(" Theta_f  |");
	printf("\n");
	
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			printf("\r");
			//print accel
			if(rc_read_accel_data(&data)<0){
				printf("read accel data failed\n");
			}
			/*printf("%6.2f %6.2f %6.2f|",    data.accel[0],\
			*				data.accel[1],\
			*				data.accel[2]);
			*/
			//print gyro 
			if(rc_read_gyro_data(&data)<0){
				printf("read gyro data failed\n");
			}
			/*printf("%6.1f %6.1f %6.1f|",	data.gyro[0]*DEG_TO_RAD,\
			*				data.gyro[1]*DEG_TO_RAD,\
			*				data.gyro[2]*DEG_TO_RAD);
			*/
			// calculate accel angle of Z over Y
			float theta_a_raw=atan2(-data.accel[2],data.accel[1]);
			// Euler integration of gyro data X 
			theta_g_raw=theta_g_raw+data.gyro[0]*dt*DEG_TO_RAD;
			printf("       %6.3f      |",theta_a_raw);
			printf("    %6.3f    |",theta_g_raw);
			rc_insert_new_ringbuf_value(&accel_in_buf,theta_a_raw);
			rc_insert_new_ringbuf_value(&gyro_in_buf,theta_g_raw);
			float accel_in_old=rc_get_ringbuf_value(&accel_in_buf,1);
			float accel_out_old=rc_get_ringbuf_value(&accel_out_buf,1);
			float gyro_in_old=rc_get_ringbuf_value(&gyro_in_buf,1);
			float gyro_out_old=rc_get_ringbuf_value(&gyro_out_buf,1);
			
			//LPF for accelerometer tf=(w*h)/(z+(w*h-1))
			float theta_a=w*dt*(accel_in_old-accel_out_old)+accel_out_old;
			//HPF for gyroscope tf= (z-1)/(z+(w*h-1))
			float theta_g=theta_g_raw-gyro_in_old-(w*dt-1.0)*gyro_out_old;
			//update buffer
			rc_insert_new_ringbuf_value(&accel_out_buf,theta_a);
			rc_insert_new_ringbuf_value(&gyro_out_buf,theta_g);
			float theta_f=theta_a+theta_g;
			//print filtered values
			printf("  %6.3f  |",theta_a);
			printf("  %6.3f  |",theta_g);
			printf("  %6.3f  |",theta_f);
			//log plotting data
			fprintf(f,"%6.3f %6.3f %6.3f\n",theta_a,theta_g,theta_f);


		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		fflush(stdout);
		// always sleep at some point
		rc_usleep(1000000/SAMPLE_RATE);
	}
	
	// exit cleanly
	fclose(f);
	rc_power_off_imu();
	rc_cleanup(); 
	return 0;
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
