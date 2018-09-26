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


// function declarations
void on_pause_pressed();
void on_pause_released();
struct Thetas get_encoder_pos();
void P_control(struct Thetas theta, float K);
// global variable declarations
struct Thetas{
		float thetaL;
		float thetaR;
};

struct Thetas Mip;
float K;
/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(int argc,char *argv[]){
	
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}
	
	// do your own initialization here
	printf("\nHello BeagleBone\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	//including command line arguments to input K and other values
	if(argc==2){ 			//argc should be 2 for proportion 	
		K=atof(argv[1]);
		printf("K is %s\n",argv[1]);
		//set_point=atof(argv[2]);
		//printf("Set point is %s\n",argv[2]);
		rc_set_state(RUNNING); 
	}
	else if(argc>2){
		printf("Too many arguments.\n");
		rc_set_state(EXITING);
	}
	else {
		printf("Two arguments expected K &Setpoint \n");
		rc_set_state(EXITING);
	}
	usleep(1000000);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			Mip=get_encoder_pos(Mip);
			printf("Left %f Right %f  \n",Mip.thetaL,Mip.thetaR);
			P_control(Mip,K);
				}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		usleep(10000);
	}
	
	// exit cleanly
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

/*
*  Get encoder positions
*  and convert to radians
*/
struct Thetas  get_encoder_pos(struct Thetas theta){
	//declare variables
	int p1;
	int p2;
	float one_rev=2*3.14159;
	p1=rc_get_encoder_pos(2);
	p2=rc_get_encoder_pos(3);
	theta.thetaL=one_rev*p1/2134;
	theta.thetaR=-one_rev*p2/2134;
return theta;					
}

void P_control(struct Thetas theta,float K){
	float duty;
	rc_enable_motors();
	duty=K*(theta.thetaR-theta.thetaL);
	printf("%f",duty);
	rc_set_motor(2,duty);
}




