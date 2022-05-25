//FINAL TIME TRIAL CODE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#include <rc/pwm.h>
#include <rc/pinmux.h>

#include <time.h> 
 
void on_pause_press();

#define CHIP 0 
#define PIX_BUF_LEN 128  
#define DEVICE_NAME	"/dev/rpmsg_pru31"
#define MSG_BYTES 2
#define MIN_PW 1000
#define MAX_PW 2000
#define GPIO_POWER_PIN 2,16
#define SERVO8 2,11
#define SERVO7 2,10

int read_pixel_buf(uint16_t *buf);
void delay(int number_of_seconds);

//double error_buf[4]; //buffer of last four errors.
//double buff_copy[4];
int pos_prev_der;
int i_state;
int i_min = -45;
int i_max = 35;

//int setA = 1;
//int setB = 0;
//int finishStop = 0;

int main()
{
	double duty_cycle;    //initialize duty_cycle variable for part 4
	int pixels_rcvd;
	uint16_t pixel_buf[PIX_BUF_LEN];
	int exit_status = EXIT_SUCCESS;
	struct pollfd pollfds[1];
	uint8_t msg[MSG_BYTES];
	int16_t pw;
	int result = 0;
	//uint16_t i=0;

	int CLK = UART1_HEADER_PIN_3;  ///< P9_26, normally UART1 RX
    int SI = UART1_HEADER_PIN_4;  ///< P9_24, normally UART1 TX



	/////////////////////////////////
	// Begin RC environment setup
	/////////////////////////////////

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return EXIT_FAILURE;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return EXIT_FAILURE;
	}
	
	 if (rc_pwm_init(0, 300) != 0) { // initialize PWM subsystem 0 (SPI header) with freq. 50 Hz for Part 4
		fprintf(stderr, "ERROR: failed to initialize PWM SS 0\n");
	}
	
	
	rc_pinmux_set(GPS_HEADER_PIN_3, PINMUX_PWM); //Set the GPS  Header pin 3 to send a PWM signal for part 4
	
	
	// setup SI and CLK pins for output
    if (rc_pinmux_set(CLK, PINMUX_GPIO) == -1)
    	{printf("Failed to set pin %d", CLK); }
    if(rc_gpio_init (CHIP, CLK, GPIOHANDLE_REQUEST_OUTPUT) ==-1)
    	{printf("Failed to init gpio pin %d", CLK); }

    if (rc_pinmux_set(SI, PINMUX_GPIO) == -1)
    	{printf("Failed to set pin %d", SI); }
    if(rc_gpio_init (CHIP, SI, GPIOHANDLE_REQUEST_OUTPUT) ==-1)
    	{printf("Failed to init gpio pin %d", SI); }

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return EXIT_FAILURE;
	}
	
	if(rc_gpio_init(GPIO_POWER_PIN, GPIOHANDLE_REQUEST_OUTPUT)==-1){
		fprintf(stderr, "ERROR in rc_gpio_init, failed to set up power rail GPIO pin\n");
		return -1;
		
	}
	if(rc_gpio_set_value(GPIO_POWER_PIN,1)==-1){
		fprintf(stderr,"ERROR in setting gpio2.16, failed to write to GPIO pin\n");
		return -1;
	}
	
	/* sets up SERVO8 for INB */
	if(rc_gpio_init(SERVO8, GPIOHANDLE_REQUEST_OUTPUT)==-1){
		fprintf(stderr, "ERROR in rc_gpio_init, failed to set up power rail GPIO pin\n");
		return -1;
		
	}
	if(rc_gpio_set_value(SERVO8,0)==-1){
		fprintf(stderr,"ERROR in setting gpio2.16, failed to write to GPIO pin\n");
		return -1;
	}
	
	/* sets up SERVO7 for INA */
	if(rc_gpio_init(SERVO7, GPIOHANDLE_REQUEST_OUTPUT)==-1){
		fprintf(stderr, "ERROR in rc_gpio_init, failed to set up power rail GPIO pin\n");
		return -1;
		
	}
	if(rc_gpio_set_value(SERVO7,1)==-1){
		fprintf(stderr,"ERROR in setting gpio2.16, failed to write to GPIO pin\n");
		return -1;
	}
	
	printf("LDO enabled\n");
	
	
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press, NULL);


	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

        /* Open the rpmsg_pru character device file */
        pollfds[0].fd = open(DEVICE_NAME, O_RDWR);


    /*
	 * If the RPMsg channel doesn't exist yet the character device won't either.
	 * Make sure the PRU firmware is loaded and that the rpmsg_pru module is inserted.
	 */
	if (pollfds[0].fd < 0) {
	    printf("Failed to open %s\n", DEVICE_NAME);
	    return -1;
	}

	/* The RPMsg channel exists and the character device is opened */
	printf("Opened %s successfully\n\n", DEVICE_NAME);

	/* Send new pulse width to PRU1 via RPMsg channel */
	pw = 1500;
	msg[0] = pw >> 8;
	msg[1] = pw & 0xff;
			
	result = write(pollfds[0].fd, msg, 2);
	if (result > 0) {
 		printf("Message sent to PRU\n");
	}

	printf("Hold pause button down for 1 second to exit\n");

	// Keep looping until state changes to EXITING

	rc_set_state(RUNNING);
	
	//memset(error_buf, 0, sizeof(error_buf));
    //memset(buff_copy, 0, sizeof(buff_copy));
	
	/*
	 *Here begins our main code where we first start reading the linescan data from
	 *our camera and ask to run the loop as long as its not exiting.
	 */
	while (rc_get_state() != EXITING) {
		pixels_rcvd = read_pixel_buf(pixel_buf);
		
		
//////////////////////////// DETECTION SCHEME #1 ////////////////////////
		
	
	char sch1_array[PIX_BUF_LEN]; //intialize array that will hold 128 '0' or '1'
	int tot_Val;  // holds the numerical value sch1_array
	int adding_ones = 0; //holds the summation of the 1's in sch1_array

	int i;
	int threshold;     //holds threshold value
	int center_index;  //holds the index of sch1_array where the center of track is
	int left_index;    //holds the index of sch1_array where the left border of the track is
	int right_index;   //holds the index of sch1_array where the right border of the track is
	int duty_to_pwm; //Here we initialize our varianble to change the duty cycle to a Pulse width value
	//int finish_count;
	


	
	threshold = 1220;//TT2: 1220 Threshold
	
	for (i=0; i<PIX_BUF_LEN; i++) {   //for loop goes through each element in pixel_buf
		if(pixel_buf[i]<threshold){   //if data less than threshold
			sch1_array[i] = '0';      //then store a '0' in sch1_array
		}else if(pixel_buf[i]>=threshold){  //if data less than threshold
			sch1_array[i] = '1';            //then store a '1' in sch1_array
			adding_ones++;
		}
	}
	
	//printf("buf stats: min(%hd), max(%hd), threshold(%hd)\n", min, max,threshold); 
	// we used the print above to see the min and max values to determine a proper value for threshold
	
	tot_Val=0; //initialize to zero
	
	for (i=0; i<PIX_BUF_LEN; i++) {//Here we increment total value to see the number of 1s in the overall character array from our linescan camera
		if(sch1_array[i] == '1'){
			tot_Val++;
		}	
	}
	
	
	
	for(i=0;i<PIX_BUF_LEN;i++){  //for loop goes through each element in sch1_array
		
		if((i > 0) & (i < 128)){ //do not analyze the first and last element
			if( sch1_array[i-1]=='0' && sch1_array[i]=='1' && sch1_array[i+1]=='1'){  
				left_index = i;	//if it is the first '1' that appers from left to right then record the index
			}else if(sch1_array[i-1]=='1' && sch1_array[i]=='1' && sch1_array[i+1]=='0'){
				right_index = i; //if it is the last '1' that appers from left to right then record the index
			}
		}
		
		center_index = (left_index + right_index) /2; //the middle of track in found by taking average of the borders. 
		
		if(i == center_index && tot_Val !=0){ //if the current index is the center of track and if there is a track then
			printf("+");                      //print a "+"
		}else {
			printf("%c",sch1_array[i]);	      //else print the element in sch1_array
		}
		
	}
	

	//code that controls LEDs and prints center index and the end.
	if(tot_Val==0){ //if there is no track
		rc_led_set(RC_LED_RED, 0);  //turn off red and green LED
		rc_led_set(RC_LED_GREEN, 0);
		printf(" center index: -"); 
	}else if((center_index>=0) & (center_index<43)){  //if track on left
		rc_led_set(RC_LED_RED, 1);  //red on, green off
		rc_led_set(RC_LED_GREEN, 0);
		printf(" center index: %d", center_index);
	}else if((center_index>=43) & (center_index<=86)){ //if track in middle
		rc_led_set(RC_LED_RED, 0);  //red off, green on
		rc_led_set(RC_LED_GREEN, 1);
		printf(" center index: %d", center_index);
	}else if((center_index>86) & (center_index<=128)){ //if track on right
		rc_led_set(RC_LED_RED, 1);  //red and green on
		rc_led_set(RC_LED_GREEN, 1);
		printf(" center index: %d", center_index);
	}
	
	
	
	double error = center_index-64;  //here we calculate our error which is the distance from the center of the track to the center of the camera
	printf("Error: %f", error);
   
	
	double err_motor = abs(center_index-64);//Here we calculate error, but for our motor 
		
	double kp_temp;//Here we intialize the Kp constant value
	
	if (center_index > 95 || center_index <35){//Here I thought of a way to reduce oscillation which is having two Kp constant terms
		kp_temp = 4.85;//Here is one for when the car is taking turns and the center of the track is further away from the camera center
	//}else if (center_index > 75 || center_index <55){
		//kp_temp = 3.5;
	}else{
		kp_temp = 2.1;//Here is another Kp value except this is smaller, it allows us to oscillate less and stay straight at a long straight track

	}
	
	/*
	
	printf("Error: %f", error);
	// I - Term
	int iTermOn = 0;//Here we intialize our I term
	if (abs(error) > 35) {//we ask if our error is very large either way, we will turn on or off our I term to allow for oscillating fixing
		iTermOn = 1;
	}
	else {
		iTermOn = 0;
	}
	
	
	
	
	*/
	
	
	double pterm  = error * kp_temp;//Here we calculate the pterm from mulitplying our error and our Kp constant
	
	double kd = 0.09;//Here is the Kd constant value we chose
	double dterm = (pos_prev_der - center_index)  * kd; //here we calculate the dterm using the previous center to the current center multiplied by the Kd constant
	pos_prev_der = center_index; //here we save the current center index of the track to be used in the next calculation

	double ki = 0;//we set to zero to test without the i term at times, it helped sometimes and other times it did not
	i_state += error; //here i_state is equal to itself plus the new error
	if (i_state > i_max) {//here we sarutate is to its max so that it does not go over its value
		i_state = i_max;
	} else if (i_state < i_min) {//here we set a minimum value so it does not continuously becoming a smaller and smaller value
		i_state = i_min;
	}
	double iterm = i_state * ki;//Here we multiply it by out Ki constant
	
	printf("Pterm: %f ", pterm);//here we print each of our terms to see how much each affects our overall pulse width calculations
	printf("Dterm: %f ", dterm);
	printf("Iterm: %f ", iterm);
	
	pw = pterm + dterm + iterm + 1550;//here we add all the p, d and i term to get our overall Pulse width value for our servo turn
	
	printf("  %d",pw);//here we print to see our overall pulse 
	
	
	if (pw > 1850){//1750 max(left turn), 1550 middle, 1170 min(right turn)
		pw = 1850;//Here I saturate our turning at 1850 since that is our maximum
	}else if (pw < 1170){
		pw = 1170;//Here I saturate our lowest pw for turning right
	} else if (pw>=1495  && pw<=1605){//and finally I implemented a deadband region so that our car will be able to stay straight as much as possible during straights
		pw = 1550;
	}
	
	
	
	
	double p;
	p = (pw * (0.000001)) * 300; //convert pulse width to duty cycle.
	rc_pwm_set_duty(0,'A',p); // Send the  new duty cycle to the PWM peripheral
		
	duty_cycle = 0.295 - (.00188)*(err_motor); //decrease 0.00## to go faster at turns, this allows us to choose its overall speed during straights and how much it slows down during turns
	
	duty_to_pwm = (duty_cycle/(2000))*1000000; //convert the duty cycle to pulse width.
	msg[0] = duty_to_pwm >> 8;
	msg[1] = duty_to_pwm & 0xff;
	result = write(pollfds[0].fd, msg, 2);
	
	//printf(" %d ",center_index-64);
	
	
	/* FINISH LINE DETECTION */
	
	
	
	for (i=0;i<(PIX_BUF_LEN-3);i++){//Here I detect a 0111 or a 1110 to find a finish line, there needs to be 6 of these so that the car will stop
		if((sch1_array[i]=='0' && sch1_array[i+1]=='1' && sch1_array[i+2]=='1' && sch1_array[i+3]=='1') 
			|| (sch1_array[i]=='1' && sch1_array[i+1]=='1' && sch1_array[i+2]=='1'&& sch1_array[i+3]=='0')){
			finish_count++;
		}
	}
		//delay(200);
	
	
	
	
	if((setA ==0 && setB == 0) || (finishStop == 1)){//This allows us to continuously stop after we have read the stop track
		rc_gpio_set_value(SERVO7,0); //INA = 0
		rc_gpio_set_value(SERVO8,0); //INB = 0
		finishStop = 1;
	} else{// otherwise if we dont see the stop line, continuously keep moving forward
		rc_gpio_set_value(SERVO7,1); //INA = 0
		rc_gpio_set_value(SERVO8,0); //INB = 0
	}
	
	
	
	if(finish_count == 6){//Here if there are 6 sides to the 3 track finish line, we set setA and setB to 0
			setA=0;
			setB=0;
	}else{//otherwise we set A to high and B to low
		setA = 1;
		setB = 0;
		finish_count=0;
	}
	


	rc_gpio_set_value(SERVO7,1); //INA = 0
	rc_gpio_set_value(SERVO8,0); //INB = 0


	
	printf("\n");
	
	// Cleanup RC environment.
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 0);
	rc_pwm_cleanup(0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST

	return exit_status;
}


/* 
 * read_pixel_buf reads a buffer of 128 linescan pixels from PRU1.
 * Each pixel is represented as a 12 bit binary value.
 */
int read_pixel_buf(uint16_t *buf) {
	/* use character device /dev/rpmsg_pru30 */
	char outputFilename[] = "/dev/rpmsg_pru30";

	/* test that /dev/rpmsg_pru30 exists */
	FILE *ofp = fopen(outputFilename, "r");

	if (ofp == NULL) {
		printf("failed to open /dev/rpmsg_pru30\n");
		return -1;
	}

	/* read voltage and channel back */
	size_t freadResult = fread(buf, sizeof(uint16_t), PIX_BUF_LEN, ofp);
	fclose(ofp);

	if (freadResult < PIX_BUF_LEN) {
		printf("Warning: failed to read a full buffer. Expected %d pixels, got %d.\n", PIX_BUF_LEN, freadResult);
	}

	return freadResult;
}


/*
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/

void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 1000000; // 1 second

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	
	printf("long press detected, exiting\n");
	
	//close(pollfds[0].fd);//bad
	
	
	rc_set_state(EXITING);
	return;
}

void delay(int number_of_seconds) 
{ 
    // Converting time into milli_seconds 
    int milli_seconds = 1000 * number_of_seconds;
  
    // Storing start time 
    clock_t start_time = clock(); 
  
    // looping till required time is not achieved 
    while (clock() < start_time + milli_seconds); 
} 