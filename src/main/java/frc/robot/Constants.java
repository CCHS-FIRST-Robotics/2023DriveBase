package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.SPI;

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
	public static final double MECANUM_WHEEL_DIAMETER 		= 0.1524; // meters

	public static double maxVelocity;

	public static final int ENCODER_CPR = 4096;
	public static final int TALON_FX_CPR = 2048;

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

	// gear ratio of the motor gearbox
	public static final double FALCON_GEARBOX_RATIO = 10.71; 

	/*
	 * Physical Constants
	 */

	// for converting motor encoder readings to standard units (rad/s)
	public static final double SPARK_MAX_CONVERSION_FACTOR = (2 * Math.PI) / 60; // base units are RPM
	public static final double TALON_CONVERSION_FACTOR = 10 * 2 * Math.PI / ENCODER_CPR; // base units are clicks per 100ms
	
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

	public static final double DRIVE_STARTING_MULTIPLIER = 0.1;

	public static final double ANALOG_DEAD_ZONE = 0.1;
	public static double LEFT_X_EXPONENT = 2;
	public static double LEFT_Y_EXPONENT = 2;
	public static double RIGHT_X_EXPONENT = 2;
	public static double RIGHT_Y_EXPONENT = 2;

	public static final double ANALOG_CROSS_DEADZONE = 0.2;

	public static final double ROBOT_WIDTH = 0.40;
	// for slowing down (stop mode)
	public static final double SLOW_DOWN_CUTOFF = 0.05;
	public static final double SLOW_DOWN_FACTOR = 2; // when slowing down, the 
	//previous speed is divided by this factor (higher = slow down faster)

	// Robot's kinematics --> cartesian location of each wheel to the physical center of the robot in meters 
	public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
	public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
	
	// public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_Y_METERS, WHEEL_ABSOLUTE_X_METERS);
	// public static final Translation2d FR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_Y_METERS, WHEEL_ABSOLUTE_X_METERS);
	// public static final Translation2d RL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_Y_METERS, -WHEEL_ABSOLUTE_X_METERS);
	// public static final Translation2d RR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_Y_METERS, -WHEEL_ABSOLUTE_X_METERS);

	public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d RL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
	public static final Translation2d RR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);

	public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(FL_WHEEL_POS, FR_WHEEL_POS, RL_WHEEL_POS, RR_WHEEL_POS);

	//  Directly from: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
	public static final double RAMSETE_B = 2.0;
	public static final double RAMSETE_ZETA = 0.7;


	// for: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryConfig.html
	// todo get values
	public static final double maxAccelerationMetersPerSecond = 1;
	public static final double maxVelocityMetersPerSecond = 1.3;

	public static final double SECONDS_BETWEEN_CODE_PERIODS = 0.02;
	
	// https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
	// max value for the falcon is 1023
	// velocity at 0.3 is around 6000
	// velocity at .25 is around 4850
	public static final double FALCON_KF = (0.3 * 1023.0) / 6000;
	// error with just kF is about +500, 0.02 is kinda arbitrary
	// significant oscillation at .1
	public static final double FALCON_KP = (0.03 * 1023.0) / 500;

	public static final double FALCON_KD = 28 * FALCON_KP;

	// kinda arbitrary
	public static final double FALCON_KI = 0.001;
	// without integral term it was within about 400 of the desired value
	public static final double FALCON_INTEGRAL_ZONE = 400;

	public static final int FALCON_PID_IDX = 0;
	public static final int FALCON_TIMEOUT_MS = 30;

	/*
	 * SENSORS 
	 */


}
