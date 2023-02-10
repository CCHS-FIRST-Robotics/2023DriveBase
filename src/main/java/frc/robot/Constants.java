package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.SPI;

/**
 * Constants - set once, and use throught the code
 * Naming convention is all caps, spaces are underscores LIKE_THIS
 */
public class Constants {

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
	public static final int FR_TALON_ID = 1; // 2
	public static final int FL_TALON_ID = 2; // 8
	public static final int RR_TALON_ID = 3; // 10
	public static final int RL_TALON_ID = 4; // 4 

	// for converting motor encoder readings to standard units (rad/s)
	public static final double SPARK_MAX_CONVERSION_FACTOR = (2 * Math.PI) / 60; // base units are RPM
	public static final double TALON_CONVERSION_FACTOR = 10 * 2 * Math.PI / ENCODER_CPR; // base units are clicks per 100ms
	
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

	public static final double ANALOG_DEAD_ZONE = 0.1;
	public static double LEFT_X_EXPONENT = 2;
	public static double LEFT_Y_EXPONENT = 2;
	public static double RIGHT_X_EXPONENT = 2;
	public static double RIGHT_Y_EXPONENT = 2;

	public static final double ROBOT_WIDTH = 0.40;
	// for slowing down (stop mode)
	public static final double SLOW_DOWN_CUTOFF = 0.05;
	public static final double SLOW_DOWN_FACTOR = 2; // when slowing down, the 
	//previous speed is divided by this factor (higher = slow down faster)

	//for limelight @avani why the fuck are your comments below the code you heathen
	public static final double LIME_HIEGHT    = 16.5;
	//height from floor to center of limelight lense in inches
	public static final double LIME_ANGLE     = 0; 
	//number of degrees from perfectly vertical
	public static final double TARGET_HIEGHT  = 1;
	//height from floor to target in inches

	public static final int SHORT_PIPE_NUM = 0;
	public static final int TALL_PIPE_NUM  = 1;
	// make sure its in the same units as the limelight vals
	public static final double PIPE_DISTANCE = ; // im aware this gives an error its a reminder to measure it lol

	Constants() {
		
	}
}
