/*****************************************************************************
* balance_config.h
*
*Settings for balance.c
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

#define SAMPLE_RATE_D1_HZ 	 100 		// inner loop speed
#define DT_D1 		  	 0.01	     	// 1/sample_rate
#define SAMPLE_RATE_D2_HZ 	 20  		// outer loop speed
#define DT_D2 		  	 0.05		// 1/sample_rate

//Physical Properties
#define MOUNT_ANGLE  	  	 0.29 
#define GEARBOX		  	 35.57
#define ENCODER_RES	  	 60
#define WHEEL_RADIUS_M	  	 0.034
#define TRACK_WIDTH_M	  	 0.035
#define V_NOMINAL	  	 7.4

// inner loop controller 100hz
#define D1_GAIN		   	 1.00000
#define D1_SATURATION_TIMEOUT	 0.4
#define FILTER_W		 0.660     	 //complementary filter frequency

//outer loop controller 20hz
#define D2_GAIN 				1.00
#define THETA_REF_MAX			.33

// electrical hookups
#define MOTOR_CHANNEL_L		 3
#define MOTOR_CHANNEL_R		 2
#define MOTOR_POLARITY_L	 1
#define MOTOR_POLARITY_R	 -1
#define ENCODER_CHANNEL_L	 3
#define ENCODER_CHANNEL_R	 2
#define ENCODER_POLARITY_L	 1
#define ENCODER_POLARITY_R	 -1

// Thread Loops
#define BATTERY_CHECK_HZ	 		5
#define SETPOINT_MANAGER_HZ   100
#define PRINTF_HZ		 					50

// other
#define TIP_ANGLE		 0.85
#define START_ANGLE		 0.3
#define START_DELAY		 0.40
#define PICKUP_DETECTION_TIME	 0.5
#endif	//BALANCE_CONFIG
