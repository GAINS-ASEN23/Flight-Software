/*
 ============================================================================
 Name        : main.cpp
 Author      : Bennett Grow, Kaylie Rick, Jason Popich
 Version     : 0.2
 Copyright   : 
 Description : Main script to run all GAINS Flight Software
 ============================================================================
 */

#include <Arduino.h>
#include <GAINSEthernet.h>
#include <kf.h>
#include <SDRW.h>


/*  OPERATING CONDITIONS  */
#define DO_CW_OR_K 1 		// 0 to use CW eqns, 1 to use kinematic eqns
#define DO_ETHERNET 0		// Perform communications over ethernet
#define DO_SD 1				// Write data to an SD card
#define DO_KF 0				// Run the Kalman Filter

/*  PIN DEFINITIONS  */
#define ADC_RES 12 			// ADC resolution (bits)
#define AP 24 				// Positive accelerometer differential pin
#define AN 25 				// Negative accelerometer differential pin
#define VT 20 				// Temperature pin from accelerometer


void setup() {

	pinMode(LED_BUILTIN, OUTPUT); // Teensy LED

	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	while (!Serial){
		;
	}

	// Set the resolution of the built-in ADC
	analogReadResolution(ADC_RES);

}


void loop() {

	Serial.printf("===============================  GAINS  ================================\n");
	Serial.printf("General Atomics Inertial Navigation System\n");
	Serial.printf("University of Colorado Boulder\n");
	Serial.printf("Ann and H.J. Smead Dept. of Aerospace Engineering\n");
	Serial.printf("Senior Design Project - Spring 2023\n");
	Serial.printf("https://github.com/GAINS-ASEN23\n");

	digitalWrite(LED_BUILTIN, HIGH);

	if (DO_ETHERNET){
		// Configure connection settings
		int local[] = {21,0,0,104};
		int localport = 8888;
		int remote[] = {21,0,0,3};
		int remoteport = 8889;
		int subnet[] = {255,255,255,0};

		// Initialize GAINSEthernet object and print connection information
		GAINSEthernet GE(local, remote, subnet, localport, remoteport);
		GE.info();
		Serial.printf("========================================================================\n");

		// Send and recieve a message over UDP
		GE.send((char*)"GAINS FSW Initialized", GE.getRemoteIP(), GE.getRemotePort());
	}

	if (DO_SD){
		delay(1000);
		Serial.println("SD - Begin");
		SDRW SD;
		SD.initFolder();

		int A_P;
		int A_N;
		int V_T;

		uint32_t start = micros();

		while (true) { //((micros() - start) <= 1200000000){ // record data for 20 minutes
			A_P = analogRead(AP);
			A_N = analogRead(AN);
			V_T = analogRead(VT);
			SD.sampleACCEL(accel(A_P, A_N), V_T);
		}
		
		Serial.println("SD - End");

	}


	if (DO_KF){
		/*****  VARIABLE DECLARATION & PREALLOCATION  *****/

		/*    Set System Variables    */

		bool contact = false;
		bool thrusting = false;

		float t0 = micros()/100000.0;			// initial time
		float t1 = t0;                          // Sample Time begin
		float t2 = t1 + 0.01;                   // Sample Time end

		uint32_t thrust_start = 0; 			    // [microseconds] Keep track of how long to record accel data
		uint32_t thrust_counter = 0;			
		float thrust_avg = 0;

		uint16_t n_u = 3;                       // Number of deterministic inputs (3 - accelerations x,y,z)
		uint16_t n_x = 6;                       // Number of States (x, y, z, x_dot, y_dot, z_dot)
		uint16_t n_z = 6;                       // Number of measured states (x_g, y_g, z_g, x_dot_g, y_dot_g, z_dot_g)

		/*    Preallocate Matricies and Vectors    */

		float *P_n_n = (float*)malloc((n_x * n_x) * sizeof(float));                     // Estimate Uncertainty Matrix
		float *x_n_n = (float*)malloc((n_x * 1) * sizeof(float));                       // Estimate State vector

		// Input Variables
		float* z_n = (float*)malloc((n_z * 1) * sizeof(float));                         // Measurement vector
		float* u_n = (float*)malloc((n_u * 1) * sizeof(float));                         // Deterministic input vector

		/********************************************************/
		/*       FILL IN THE INITIAL VECTORS AND MATRICIES      */
		/********************************************************/

		/*    Set gravitational parameters    */

		float mu_moon = 4.9048695e12;           // Gravitational parameter of the Moon [m^3 s^-2]
		float rad_moon = 1737447.78;            // Radius of the Moon [m]
		float orbit_alt = 50000.0;              // Orbit radius [m]

		// Set the mean motion for CW equations
		float n = calculate_mean_motion(mu_moon, rad_moon, orbit_alt);

		/*    Set the initial orbit initial conditions    */

		float alpha = 0;                    // The Phase Angle Alpha [deg]
		float beta = 0;                     // The Phase Angle Beta [deg]
		float deviation = 50000;            // The deviation in position from the chief satellite [m]

		set_cw_ics(x_n_n, alpha, beta, deviation, n);

		/*    Set the initial uncertainty matrix    */

		float sigma_position = 1000;        // The error in the position x,y,z [m]
		float sigma_velocity = 0.1;         // The error in the velocity x,y,z [m/s]

		// Make the P_n_n matrix an identity matrix
		eye(P_n_n, n_x);

		// Populate sigmas
		float sigma_p_n_n[6] = {sigma_position, sigma_position, sigma_position, sigma_velocity, sigma_velocity, sigma_velocity};
		set_p_ic(n_x, P_n_n, sigma_p_n_n);

		// Create the KF object
		KalmanFilter KF(n_x, n_u, n_z, x_n_n, P_n_n, DO_CW_OR_K);

		/*  Set the measurement covariance matrix   */
		float sigma_position_ground = 1000;
		float sigma_velocity_ground = 0.1;

		// Populate the sigmas
		float sigma_r_n[6] = {sigma_position_ground, sigma_position_ground, sigma_position_ground, sigma_velocity_ground, sigma_velocity_ground, sigma_velocity_ground};
		KF.set_r_n(sigma_r_n);

		/*  Set the initial input covariance */
		float sigma_accel_x = 1;
		float sigma_accel_y = 1;
		float sigma_accel_z = 1;

		// Populate sigmas
		float sigma_q_a[3] = {sigma_accel_x, sigma_accel_y, sigma_accel_z};
		KF.set_q_a(sigma_q_a);

		/*  Main Loop */
		while (true)
		{
			//GE.read();

			// Set the measurement vector, if ground contact is non-zero
			if (contact){
				z_n[0] = 0;
				z_n[1] = 0;
				z_n[2] = 0;
				z_n[3] = 0;
				z_n[4] = 0;
				z_n[5] = 0;
				KF.set_mea_input_vector(z_n);
			}

			// Set the H matrix as if we don't have a ground contact, but if contact set true
			KF.set_h(contact);

			// Set the deterministic input vector, if thrusting is non-zero
			if (thrusting){
				// Reset variables for this timestep of thrusting
				thrust_start = micros();
				thrust_counter = 0;
				thrust_avg = 0;

				// Average samples for a desired time in microseconds
				while ((micros() - thrust_start) <  5000){
					int A_P = analogRead(AP);
					int A_N = analogRead(AN);
					thrust_avg = thrust_avg + accel(A_P, A_N);
					thrust_counter++;
				}
				thrust_avg = thrust_avg/thrust_counter;

				// NEED TO CORRECT FOR BIAS, MISALIGNMENT, ETC. HERE

				u_n[0] = thrust_avg;
				u_n[1] = 0;
				u_n[2] = 0;
				KF.set_det_input_vector(u_n);
			}

			// Run the KF
			KF.KF_run(t1, t2, n);

			// Delay depending on requirements
			
			// Update t1 and t2
			t2 = t1;
			t1 = (micros()/100000.0) - t0;

		}
	}



	digitalWrite(LED_BUILTIN, LOW);
	exit(0);
}