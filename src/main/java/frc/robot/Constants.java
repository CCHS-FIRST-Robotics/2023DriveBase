package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utils.R2Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * Constants - set once, and use throught the code
 * Naming convention is all caps, spaces are underscores LIKE_THIS
 */
public class Constants {

	public static final boolean ROBOT_START_CENTER_FIELD = false;

	// the port for the xbox controller
	public static final int XBOX_CONTROLLER_PORT 			= 0;
	public static final int XBOX_CONTROLLER_ALTERNATE_PORT 			= 1; 

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
	public static final int FR_TALON_ID = 1; // 2
	public static final int FL_TALON_ID = 2; // 8
	public static final int RR_TALON_ID = 3; // 10
	public static final int RL_TALON_ID = 4; // 4 

	public static final int SHOULDER_TALON_ID = 20;
	public static final int ELBOW_TALON_ID = 10;

	public static final int SHOULDER_FALCON_ID = 5;
	public static final int ELBOW_FALCON_ID = 6;

	// gear ratio of the motor gearbox
	public static final double FALCON_GEARBOX_RATIO = 10.71; 

	/*
	 * Physical Constants
	 */
	public static final int LIMIT_SWITCH_ID = 4; // DIO port num

	// for converting motor encoder readings to standard units (rad/s)
	public static final double SPARK_MAX_CONVERSION_FACTOR = (2 * Math.PI) / 60; // base units are RPM
	public static final double TALON_CONVERSION_FACTOR = 10 * 2 * Math.PI / ENCODER_CPR; // base units are clicks per 100ms
	public static final double METERS_TO_FALCON_CLICKS = TALON_FX_CPR * FALCON_GEARBOX_RATIO / (Math.PI * MECANUM_WHEEL_DIAMETER);
	
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

	public static final double DRIVE_STARTING_MULTIPLIER = 0.3;

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

	// RAMP CONTROL CONSTANTS TODO: NEEDS TO BE TUNED
	public static final double RAMP_P = .05;
	public static final double RAMP_I = 0;
	public static final double RAMP_D = 0; // .001
	
	public static final double RAMP_G = .3;

	// ALIGN CONTROL CONSTANTS
	public static final double ALIGN_P = .5;
	public static final double DISTANCE_FROM_APRILTAG_TO_CONE = .5; // meters


	//  Directly from: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
	public static final double RAMSETE_B = 2.0;
	public static final double RAMSETE_ZETA = 0.7;


	// for: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryConfig.html
	// todo get values
	public static final double maxAccelerationMetersPerSecond = 1;
	public static final double maxVelocityMetersPerSecond = 1.3;

	public static final double SECONDS_BETWEEN_CODE_PERIODS = 0.02;
	
	// LIMELIGHT //
	public static final double SHORT_TARGET_HEIGHT  = .56 + .0508;
	public static final double TALL_TARGET_HEIGHT  = 1.05 + .0508;

	public static final int SHORT_PIPE_NUM = 0;
	public static final int TALL_PIPE_NUM = 1;

	public static final double LIME_HEIGHT = .42;
	public static final double LIME_ANGLE = 0;

	// arm lengths in meters
	public static final double UPPER_ARM_LENGTH = .68; 
	public static final double LOWER_ARM_LENGTH = 0.705;
	public static final double WRIST_LENGTH = .26;

	public static final double MIN_ARM_EXTENSION = UPPER_ARM_LENGTH - LOWER_ARM_LENGTH;
	public static final double MAX_ARM_EXTENSION = UPPER_ARM_LENGTH + LOWER_ARM_LENGTH;

	// Distance from each arm joint to the linkage's center of mass
	public static final double UPPER_ARM_COM_DIST = .7; // meters
	public static final double LOWER_ARM_COM_DIST = .2; // meters
	static double densityOfMetalLower = .347 / .405;
	static double densityOfMetalUpper = .623 / 1.04;
	static double massOfMetalLower = densityOfMetalLower * LOWER_ARM_LENGTH;
	static double massOfMetalUpper = densityOfMetalUpper * UPPER_ARM_LENGTH;
	static double chainMass = .85 * .53;
	static double falconMass = 1.33;
	static double clawMass = 1.86;
	public static final double UPPER_ARM_MASS = massOfMetalUpper + clawMass;
	public static final double LOWER_ARM_MASS = massOfMetalLower + chainMass + falconMass;

	// Mass of each arm segment in kg
	// public static final double UPPER_ARM_WEIGHT = 2.25 + 1.4; // kg
	// public static final double LOWER_ARM_WEIGHT = 4.5; // kg

	//Height of the shoulder joint from the floor in meters
	public static final double SHOULDER_JOINT_HEIGHT = .59;

	public static final int CLAW_FORWARD_NUM = 0;
	public static final int CLAW_BACKWARD_NUM = 1;
	public static final int WRIST_FORWARD_NUM = 2;
	public static final int WRIST_BACKWARD_NUM = 3;

	// Shoulder PID and Feedfoward gains for positional control
	public static final double SHOULDER_KP = 1.2; // 1.4
	public static final double SHOULDER_KI = 0.00; // .05
	public static final double SHOULDER_KD = 0.00; //.01

	public static final double SHOULDER_KS = 0;
	public static final double SHOULDER_KG = .9;
	public static final double SHOULDER_KV = 0;
	public static final double SHOULDER_KA = 0;

	// Elbow PID and Feedfoward gains
	public static final double ELBOW_KP = 1.1; // 1.4
	public static final double ELBOW_KI = 0;
	public static final double ELBOW_KD = 0.00; //.01

	public static final double ELBOW_KS = 0;
	public static final double ELBOW_KG = 0.7;
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

	public static final double ARM_MAX_SPEED = .5; // meters per second
	public static final double ARM_MAX_ACCELERATION = .5; // meters per second per second

	// When you push the controller all the way in one direction, the max forward is what we set to PID loop to
	// TODO: name for that probably isnt intuitive so anyone got suggestions?
	public static final double MAX_FORWARD_X = .005; // meters
	public static final double MAX_FORWARD_Y = .005; // meters

	public static final double WRIST_MIN_ACTUATE = .7;
	public static final double WRIST_MAX_ACTUATE = .8;

	// hard min/max x and y values for the arm
	public static final double minX = 0.2; // slightly less than the furthest possible distance to make sure we don't lose a DOF
	public static final double maxX = 1.3; 
	public static final double minY = 0.4;
	public static final double maxY = 1.9; // 6'6"

	public static boolean isInFrameX(double x) {
		return (Math.abs(x) < .35); // frame is ~1 meter long
	}

	public static boolean isBelowElectricalBoard(double y) {
		return (y < .6); // top of electrical board is ~0.6 meters off the ground
	}

	public static boolean isBelowFrame(double y) {
		return (y < .2);
	}

	public static boolean isZero(double x) {
		return Math.abs(x) < 2 * Double.MIN_VALUE;
	}

	// alpha is the shoulder joint angle in relation to the horizontal
	// beta is the elbow joint angle in relation to the horzinontal
	// angles in degrees
	public static final double minAlpha = -30;
	public static final double maxAlpha = 170;

	public static final double minBeta = -175;
	public static final double maxBeta = 260;

	// Positions for the arm
	public static enum ArmFixedPosition {
		CUBE_LOWER,
		CUBE_HIGHER,
		CONE_LOWER,
		CONE_HIGHER_PRE_POS,
		CONE_HIGHER,
		DROPOFF_LOW,
		PICKUP_GROUND,
		PICKUP_GROUND_LAYING_DOWN,
		PICKUP_SUBSTATION,
		NEUTRAL
	} 
	public static final R2Vector CUBE_LOWER = new R2Vector(0.72, 1.04);
	public static final R2Vector CUBE_HIGHER = new R2Vector(1.25, 1.35);
	public static final R2Vector CONE_LOWER = new R2Vector(0.74, 1.3);
	public static final R2Vector CONE_HIGHER_PRE_POS = new R2Vector(.4, 1.55);
	public static final R2Vector CONE_HIGHER = new R2Vector(1.05, 1.55);
	public static final R2Vector DROPOFF_LOW = new R2Vector(.6, .5);
	public static final R2Vector PICKUP_GROUND = new R2Vector(.7, .3);
	public static final R2Vector PICKUP_GROUND_LAYING_DOWN = new R2Vector(.75, .45);
	public static final R2Vector PICKUP_SUBSTATION = new R2Vector(.35, 1.15);
	public static final R2Vector NEUTRAL = new R2Vector(.25, .7);

	// https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#calculating-velocity-feed-forward-gain-kf
	// max value for the falcon is 1023
	// velocity at 0.4 is around 7600
	public static final double FALCON_KF = (0.4 * 1023.0) / 7637.625;
	// error with just kF is about +400, 0.4 is kinda arbitrary
	// significant oscillation at .1
	public static final double FALCON_KP = (0.4 * 1023.0) / 400;

	public static final double FALCON_KD = 15 * FALCON_KP;

	// kinda arbitrary
	public static final double FALCON_KI = 0.006;
	// without integral term it was within about 50 of the desired value
	public static final double FALCON_INTEGRAL_ZONE = 50;

	public static final int FALCON_PID_IDX = 0;
	public static final int FALCON_TIMEOUT_MS = 30;

	/*
	 * SENSORS 
	 */


}
