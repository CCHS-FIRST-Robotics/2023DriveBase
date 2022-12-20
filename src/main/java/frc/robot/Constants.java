package frc.robot;

import java.lang.Math;

/**
 * Constants - set once, and use throught the code
 * Naming convention is all caps, spaces are underscores LIKE_THIS
 */
public class Constants {
	// TODO: add actual values for all constants (current values are placeholder)

	// the port for the xbox controller
	public static final int XBOX_CONTROLLER_PORT 			= 0; 

	public static final double DRIVE_MAX_ANGULAR_VELOCITY 	= 2 * Math.PI;
	
	public static final double TANK_WHEEL_RADIUS 			= 0.075; // meters

	public static double maxVelocity;

	public static final int ENCODER_CPR = 4096;

	// the amount of time (in seconds) between calls of the periodic function
	public static final double dt = 0.02;

	// for tank drive
	public static final int LEFT_VICTOR_ID = 7;
	public static final int RIGHT_VICTOR_ID = 4;
	public static final int TALON_ID = 9;
	public static final int SPARK_MAX_ID = 12;
	
	// for mecanum drive FR = front right, FL = front left, RR = rear right, RL = rear left
	public static final int FR_TALON_PORT = 2;
	public static final int FL_TALON_PORT = 8;
	public static final int RR_TALON_PORT = 10;
	public static final int RL_TALON_PORT = 4;
	
	public static final double ANALOG_DEAD_ZONE = 0.05;

	public static final double ROBOT_WIDTH = 0.40;

	Constants() {
		
	}
}
