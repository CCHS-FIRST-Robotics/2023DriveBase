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
	public static final double PERIOD = 0.02;

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

	public static final int SHOULDER_TALON_ID = 3;
	public static final int ELBOW_TALON_ID = 5;
	public static final int ELBOW_FALCON_ID = 9;


	public static final int SHOULDER_TALON_ID = 3;
	public static final int ELBOW_TALON_ID = 5;
	public static final int ELBOW_FALCON_ID = 9;


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
	public static double RIGHT_X_EXPONENT = 1.5;
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
	
	// arm lengths in meters
	public static final double UPPER_ARM_LENGTH = 1.05; 
	public static final double LOWER_ARM_LENGTH = 0.72;
	public static final double WRIST_LENGTH = 0;

	// Distance from each arm joint to the linkage's center of mass
	public static final double UPPER_ARM_COM_DIST = .7; // meters
	public static final double LOWER_ARM_COM_DIST = .36; // meters

	// Mass of each arm segment in kg
	public static final double UPPER_ARM_WEIGHT = 2.25 + 1.4; // kg
	public static final double LOWER_ARM_WEIGHT = 4.5; // kg

	//Height of the shoulder joint from the floor in meters
	public static final double SHOULDER_JOINT_HEIGHT = .59;

	// Shoulder PID and Feedfoward gains for positional control
	public static final double SHOULDER_KP = .4;
	public static final double SHOULDER_KI = 0;
	public static final double SHOULDER_KD = 0.01;

	public static final double SHOULDER_KS = 0;
	public static final double SHOULDER_KG = 1.4;
	public static final double SHOULDER_KV = 0;
	public static final double SHOULDER_KA = 0;

	// Elbow PID and Feedfoward gains
	public static final double ELBOW_KP = .2;
	public static final double ELBOW_KI = 0;
	public static final double ELBOW_KD = 0;

	public static final double ELBOW_KS = 0;
	public static final double ELBOW_KG = 1.5;
	public static final double ELBOW_KV = 0;
	public static final double ELBOW_KA = 0;

	// Shoulder PID and Feedfoward gains for velocity control
	public static final double SHOULDER_VELOCITY_KP = 0;
	public static final double SHOULDER_VELOCITY_KI = 0;
	public static final double SHOULDER_VELOCITY_KD = 0;

	// Elbow PID and Feedfoward gains
	public static final double ELBOW_VELOCITY_KP = 0;
	public static final double ELBOW_VELOCITY_KI = 0;
	public static final double ELBOW_VELOCITY_KD = 0;

	// Max velocity and acceleration for the arm
	public static final double SHOULDER_MAX_VELOCITY = 45; // deg per second
	public static final double SHOULDER_MAX_ACCELERATION = 90; // deg per second^2

	public static final double ELBOW_MAX_VELOCITY = 45; // deg per second
	public static final double ELBOW_MAX_ACCELERATION = 90; // deg per second^2

	public static final double ARM_MAX_SPEED = 1; // meters per second
	public static final double ARM_MAX_ACCELERATION = 1; // meters per second

	// When you push the controller all the way in one direction, the max forward is what we set to PID loop to
	// TODO: name for that probably isnt intuitive so anyone got suggestions?
	public static final double MAX_FORWARD_X = .5; // meters
	public static final double MAX_FORWARD_Y = .3; // meters

	// hard min/max x and y values for the arm
	public static final double minX = -1.8; // slightly less than the furthest possible distance to make sure we don't lose a DOF
	public static final double maxX = 1.8; 
	public static final double minY = 0.05;
	public static final double maxY = 1.95; // 6'6"

	public static boolean isInFrameX(double x) {
		return (Math.abs(x) < .5); // frame is ~1 meter long
	}

	public static boolean isBelowElectricalBoard(double y) {
		return (y < .6); // top of electrical board is ~0.6 meters off the ground
	}

	public static boolean isBelowFrame(double y) {
		return (y < .2);
	}

	// alpha is the shoulder joint angle in relation to the horizontal
	// beta is the elbow joint angle in relation to the horzinontal
	// angles in degrees
	public static final double minAlpha = -30;
	public static final double maxAlpha = 170;

	public static final double minBeta = -85;
	public static final double maxBeta = 240;

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
