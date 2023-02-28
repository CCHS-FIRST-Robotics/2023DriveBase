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
	public static final int XBOX_CONTROLLER_ALTERNATE_PORT 			= 1; 

	public static final double DRIVE_MAX_ANGULAR_VELOCITY 	= 2 * Math.PI;
	
	public static final double TANK_WHEEL_RADIUS 			= 0.075; // meters

	public static double maxVelocity;

	public static final int ENCODER_CPR = 4096;

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

	public static final int LIMIT_SWITCH_ID = 4; // DIO port num

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

	// LIMELIGHT //
	public static final double SHORT_TARGET_HEIGHT  = .56 + .0508;
	public static final double TALL_TARGET_HEIGHT  = 1.05 + .0508;

	public static final int SHORT_PIPE_NUM = 0;
	public static final int TALL_PIPE_NUM = 1;

	public static final double LIME_HEIGHT = .42;
	public static final double LIME_ANGLE = 0;

	// arm lengths in meters
	public static final double UPPER_ARM_LENGTH = .68; 
	public static final double LOWER_ARM_LENGTH = 0.71;
	public static final double WRIST_LENGTH = .26;

	// Distance from each arm joint to the linkage's center of mass
	public static final double UPPER_ARM_COM_DIST = .7; // meters
	public static final double LOWER_ARM_COM_DIST = .36; // meters

	// Mass of each arm segment in kg
	public static final double UPPER_ARM_WEIGHT = 2.25 + 1.4; // kg
	public static final double LOWER_ARM_WEIGHT = 4.5; // kg

	//Height of the shoulder joint from the floor in meters
	public static final double SHOULDER_JOINT_HEIGHT = .59;

	public static final int CLAW_FORWARD_NUM = 0;
	public static final int CLAW_BACKWARD_NUM = 1;
	public static final int WRIST_FORWARD_NUM = 2;
	public static final int WRIST_BACKWARD_NUM = 3;

	// Shoulder PID and Feedfoward gains for positional control
	public static final double SHOULDER_KP = .6;
	public static final double SHOULDER_KI = 0;
	public static final double SHOULDER_KD = .02;

	public static final double SHOULDER_KS = 0;
	public static final double SHOULDER_KG = 1.55;
	public static final double SHOULDER_KV = 0;
	public static final double SHOULDER_KA = 0;

	// Elbow PID and Feedfoward gains
	public static final double ELBOW_KP = .2;
	public static final double ELBOW_KI = 0;
	public static final double ELBOW_KD = 0;

	public static final double ELBOW_KS = 0;
	public static final double ELBOW_KG = .7;
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
	public static final double ARM_MAX_ACCELERATION = 1; // meters per second

	// When you push the controller all the way in one direction, the max forward is what we set to PID loop to
	// TODO: name for that probably isnt intuitive so anyone got suggestions?
	public static final double MAX_FORWARD_X = 1; // meters
	public static final double MAX_FORWARD_Y = .3; // meters

	// hard min/max x and y values for the arm
	public static final double minX = -1.45; // slightly less than the furthest possible distance to make sure we don't lose a DOF
	public static final double maxX = 1.45; 
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

	public static final double minBeta = -120;
	public static final double maxBeta = 240;

	Constants() {
		
	}
}