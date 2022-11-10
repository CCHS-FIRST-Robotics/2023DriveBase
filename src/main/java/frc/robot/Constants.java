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
	public static final double DRIVE_WHEEL_RADIUS 			= 0.1; // meters

	public static double maxVelocity;

	// for tank drive
	public static final int LEFT_TALON_PORT  = Integer.MAX_VALUE;
	public static final int RIGHT_TALON_PORT = Integer.MIN_VALUE;
	
	// for mecanum drive FR = front right, FL = front left, RR = rear right, RL = rear left
	public static final int FR_TALON_PORT = 2;
	public static final int FL_TALON_PORT = 8;
	public static final int RR_TALON_PORT = 10;
	public static final int RL_TALON_PORT = 4;
	
	public static final double ANALOG_DEAD_ZONE = 0.05;


	Constants() {
		maxVelocity = DRIVE_MAX_ANGULAR_VELOCITY * DRIVE_WHEEL_RADIUS;
		
	}
}
