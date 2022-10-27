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

	public static final int LEFT_TALON_PORT 	= Integer.MAX_VALUE;
	public static final int RIGHT_TALON_PORT 	= Integer.MIN_VALUE;

	Constants() {
		maxVelocity = DRIVE_MAX_ANGULAR_VELOCITY * DRIVE_WHEEL_RADIUS;
		
	}
}
