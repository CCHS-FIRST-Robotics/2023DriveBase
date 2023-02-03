package frc.robot;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

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
	public static final int FR_TALON_PORT = 1; // 2
	public static final int FL_TALON_PORT = 2; // 8
	public static final int RR_TALON_PORT = 3; // 10
	public static final int RL_TALON_PORT = 4; // 4

	public static final int FR_ENCODER_ID = 1;
	public static final int FL_ENCODER_ID = 2;
	public static final int RR_ENCODER_ID = 3;
	public static final int RL_ENCODER_ID = 4;


	/*
	 * Physical Constants
	 */

	// for converting motor encoder readings to standard units (rad/s)
	public static final double SPARK_MAX_CONVERSION_FACTOR = (2 * Math.PI) / 60; // base units are RPM
	public static final double TALON_CONVERSION_FACTOR = 10 * 2 * Math.PI / ENCODER_CPR; // base units are clicks per 100ms
	
	public static final double ANALOG_DEAD_ZONE = 0.1;

	public static final double ANALOG_CROSS_DEADZONE = 0.2;

	public static final double ROBOT_WIDTH = 0.40;
	// for slowing down (stop mode)
	public static final double SLOW_DOWN_CUTOFF = 0.05;
	public static final double SLOW_DOWN_FACTOR = 2; // when slowing down, the 
	//previous speed is divided by this factor (higher = slow down faster)

	// Robot's kinematics --> cartesian location of each wheel to the physical center of the robot in meters 
	public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
	public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
	
	public static final Translation2d FL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d RL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d RR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);

	public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(FL_WHEEL_POS, FR_WHEEL_POS, RL_WHEEL_POS, RR_WHEEL_POS);


	/*
	 * SENSORS 
	 */


}
