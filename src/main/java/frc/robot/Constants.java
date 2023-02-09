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
	public static final int FR_TALON_ID = 2; // 1
	public static final int FL_TALON_ID = 8; // 2
	public static final int RR_TALON_ID = 10; // 3
	public static final int RL_TALON_ID = 4; // 4 

	public static final int SHOULDER_TALON_ID = 0;
	public static final int ELBOW_TALON_ID = 5;

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

	// arm lengths in meters
	//TODO: UPDATE ARM VALUES
	public static final double UPPER_ARM_LENGTH = 0.97; 
	public static final double LOWER_ARM_LENGTH = 0.71;
	public static final double WRIST_LENGTH = 0;

	// Shoulder PID and Feedfoward gains
	public static final double SHOULDER_KP = 0;
	public static final double SHOULDER_KI = 0;
	public static final double SHOULDER_KD = 0;

	public static final double SHOULDER_KS = 0;
	public static final double SHOULDER_KG = 0;
	public static final double SHOULDER_KV = 0;
	public static final double SHOULDER_KA = 0;

	// Elbow PID and Feedfoward gains
	public static final double ELBOW_KP = 0;
	public static final double ELBOW_KI = 0;
	public static final double ELBOW_KD = 0;

	public static final double ELBOW_KS = 0;
	public static final double ELBOW_KG = 0;
	public static final double ELBOW_KV = 0;
	public static final double ELBOW_KA = 0;

	// TODO: set values
	public static final double minX = 0;
	public static final double maxX = 120;
	public static final double minY = 0;
	public static final double maxY = 200;

	// alpha is the shoulder joint angle in relation to the horizontal
	// beta is the elbow joint angle in relation to the horzinontal
	// angles in degrees
	public static final double minAlpha = -30;
	public static final double maxAlpha = 170;

	public static final double minBeta = -75;
	public static final double maxBeta = 240;

	Constants() {
		
	}
}
