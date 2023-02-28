package frc.robot.subsystems;
import frc.robot.*;
import frc.robot.utils.*;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;

public class Arm {

    // define motor and encoder objects
	WPI_TalonSRX shoulderMotor, elbowMotorEncoder;
	WPI_TalonFX elbowMotor;
	// TalonFXSensorCollection shoulderFalconSensor, elbowFalconSensor;

	// ProfiledPIDController shoulderPID, elbowPID;
	PIDController shoulderPID, elbowPID;
	ArmFeedforward shoulderFeedforward, elbowFeedforward;

	PIDController shoulderVelocityPID, elbowVelocityPID;

	DigitalInput limitSwitch;

	double lastShoulderAngle, lastElbowAngle;

	// PID constants when tuning - TESTING ONLY
	public double shoulderP, shoulderI, shoulderD;
	public double elbowP, elbowI, elbowD;

	// Toggles whether motor limits are active - displayed on shuffleboard
	public boolean motorLimits = true;

	// Toggles whether the motors should stop - controlled manually
	public boolean manualMotorStop = false;

    /**
	 * Constructor for Arm Class
	 * 
	 * @param shoulderTalonPort
	 * @param elbowTalonPort
	 */
	public Arm(int shoulderTalonPort, int elbowTalonPort, int elbowFalconPort, DigitalInput limitSwitch) {
		// motor docs lol: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
		
		// initialize motors
        shoulderMotor = new WPI_TalonSRX(shoulderTalonPort);
        elbowMotorEncoder = new WPI_TalonSRX(elbowTalonPort);

		// initialize Falcon motors (USE LATER)
        // shoulderMotor = new WPI_TalonFX(shoulderTalonPort);
        elbowMotor = new WPI_TalonFX(elbowFalconPort);
		

		// set the config to default in case there's something else I'm missing
		shoulderMotor.configFactoryDefault();
		elbowMotor.configFactoryDefault();

		// setup motor encoders
		TalonSRXConfiguration config = new TalonSRXConfiguration();
		config.peakCurrentLimit = 40; // the peak current, in amps
		config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
		config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
		shoulderMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
		// TODO: this isnt wokring with falcons
		// elbowMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

		// https://github.com/GwhsRobotics3/Team-5507-2018/blob/b4d3e1d5e899132185e9f7b9711d5a92f322d659/src/org/usfirst/frc/team5507/robot/subsystems/DriveTrain.java#L112
		shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		this.limitSwitch = limitSwitch;

		initControllers(false);

		// shoulderMotor.setNeutralMode(NeutralMode.Brake);
		// elbowMotor.setNeutralMode(NeutralMode.Brake);
    }

	/**
	 * initalizes controllers in debug mode - uses PID constants from shuffleboard
	 */
	public void initControllers() {
		// Positional PID Controllers
        // shoulderPID = new ProfiledPIDController(shoulderP, shoulderI, shoulderD,
		// 	new TrapezoidProfile.Constraints(Constants.SHOULDER_MAX_VELOCITY, Constants.SHOULDER_MAX_ACCELERATION)
		// );
        // elbowPID = new ProfiledPIDController(elbowP, elbowI, elbowD,
		// 	new TrapezoidProfile.Constraints(Constants.ELBOW_MAX_VELOCITY, Constants.ELBOW_MAX_ACCELERATION)
		// );
		
		// Velocity PID Controllers
		shoulderVelocityPID = new PIDController(Constants.SHOULDER_VELOCITY_KP, Constants.SHOULDER_VELOCITY_KI, Constants.SHOULDER_VELOCITY_KD);
        elbowVelocityPID = new PIDController(Constants.ELBOW_VELOCITY_KP, Constants.ELBOW_VELOCITY_KI, Constants.ELBOW_VELOCITY_KD);
	}

	/**
	 * Initializes PID controllers
	 * 
	 * @param debug (boolean) - whether or not we're in "debug" mode for tuning PID constants
	 */
	private void initControllers(boolean debug) {
		if (debug) {
			initControllers();
			return;
		}

		// Positional PID Controllers
        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD
			// new TrapezoidProfile.Constraints(Constants.SHOULDER_MAX_VELOCITY, Constants.SHOULDER_MAX_ACCELERATION)
		);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD
			// new TrapezoidProfile.Constraints(Constants.ELBOW_MAX_VELOCITY, Constants.ELBOW_MAX_ACCELERATION)
		);
		// Velocity PID Controllers
		shoulderVelocityPID = new PIDController(Constants.SHOULDER_VELOCITY_KP, Constants.SHOULDER_VELOCITY_KI, Constants.SHOULDER_VELOCITY_KD);
        elbowVelocityPID = new PIDController(Constants.ELBOW_VELOCITY_KP, Constants.ELBOW_VELOCITY_KI, Constants.ELBOW_VELOCITY_KD);
	}

	/**
	 * toggles whether motor limits are activated - called when X and Y are pressed simultaneously on the xbox controller
	 */
	public void toggleMotorCheck() {
		motorLimits = !motorLimits;
	}

	/**
	 * Toggles whether the motors are manually stopped - called when B is pressed on the xbox controller
	 */
	public void toggleManualMotorStop() {
		manualMotorStop = !manualMotorStop;
	}

	public boolean shouldMotorStop() {
		return false;

		// double alpha = getShoulderAngle();
		// double beta = getElbowAngle();
		// double theta = getWristAngle();

		// boolean motorStop = Kinematics.shouldMotorStop(alpha, beta, theta);

		// return (motorStop && motorLimits) || manualMotorStop; // check if the motor limits are activated or if driver is trying to stop them manually
	}

	/**
	 * Sets the motor outputs to resist gravity and sets the motors to brake mode
	 */
	public void stopMotors() {
		// System.out.println(getElbowFeedforward());
		
		// shoulderMotor.setVoltage(getShoulderFeedforward());
		// elbowMotor.setVoltage(getElbowFeedforward());
		setShoulder(lastShoulderAngle);
		setElbow(lastElbowAngle);

		shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * Returns the voltage needed to counteract gravity
	 * 
	 * @return controlInput (double) voltage to send to motors
	 */
	public double getElbowFeedforward() {
		return -Constants.ELBOW_KG * Math.cos(Math.toRadians(getElbowAngle()));
	}


	//TODO: FUNCTION RETURNS A CONSTANTS VALUE LOL SOMEONE NEEDS TO FIX THAT
	/**
	 * Calculates the COM of the arm and returns the voltage needed to counteract gravity
	 * Diagram: https://raw.githubusercontent.com/CCHS-FIRST-Robotics/2023DriveBase/main/images/B6D091A1-433E-4F3A-8475-E74F224E33DC.png
	 * 
	 * @return controlInput (double) voltage to send to motors 
	 */
	public double getShoulderFeedforward() {
		double d1 = Constants.UPPER_ARM_COM_DIST;
		double d2 = Constants.LOWER_ARM_COM_DIST;
		double l1 = Constants.LOWER_ARM_LENGTH;

		double alpha = Math.toRadians(getShoulderAngle());
		double beta = Math.toRadians(getElbowAngle());

		double comX = (d1*Math.cos(alpha) + l1*Math.cos(alpha) + d2*Math.cos(beta)) / 2;
		double comY = (d1*Math.cos(alpha) + l1*Math.cos(alpha) + d2*Math.cos(beta)) / 2;
		
		// TODO: using atan2 might fix the sign error, have to test
		double controlInput = -Constants.SHOULDER_KG * Math.cos(Math.atan(comY / comX));
		// atan returns between -pi/2 and pi/2, but cos only returns pos values on that interval, so we have to check the sign manually
		if (alpha > Math.PI / 2) {
			return -controlInput;
		} else {
			return controlInput;
		}
	}

	public double getShoulderRawAngle() { 
		return shoulderMotor.getSelectedSensorPosition();
	}

	/**
	 * @return angle (double) degrees of the first linkage from the horizontal
	 */
	public double getShoulderAngle() {
		// encoder reads in [-2048, 2048] god knows why it's not the same as the other
		double angle = 360 - (shoulderMotor.getSelectedSensorPosition(1) + 2048) * 360/4096 - 86; // prints the position of the selected sensor
		return angle;
	}

	/**
	 * @return angle (double) degrees of the second linkage from the horizontal
	 */
	public double getElbowAngle() {
		// encoder reads in [-4096, 0], and absolute position is off by 10 degrees 
		// offset  by shoulder angle so that the angle is relative to the horizotal
		double angle = getShoulderAngle() - ((elbowMotorEncoder.getSelectedSensorPosition(1) + 1300) * 360/4096) - 47;
		return angle;
	}

	public void updatePrevAngles() {
		if (!Kinematics.shouldMotorStop(getShoulderAngle(), getElbowAngle(), getWristAngle())) { 
			lastShoulderAngle = getShoulderAngle();
			lastElbowAngle = getElbowAngle();
		}
	}

	/**
	 * @return angle (double) degrees of the third linkage (claw) from the horizontal
	 */
	public double getWristAngle() {
		int wristActuated = isWristActuated() ? 1:0;
		return getElbowAngle() + wristActuated * 90;
	}

	// TODO: write method
	public boolean isWristActuated() {
		return false;
	}

	/**
	 * @return angle (double) angular velocity of the shoulder joint - deg/s
	 */
	public double getShoulderAngularVelocity() {
		return shoulderMotor.getSelectedSensorVelocity() * 360/4096 * 10;
	}

	/**
	 * @return angle (double) angular velocity of the elbow joint - deg/s
	 */
	public double getElbowAngularVelocity() {
		return shoulderMotor.getSelectedSensorVelocity() * 360/4096 * 10;
	}

	// /**
	//  * Updates the angular velocity of the shoulder -- must be called every 1ms
	//  */
	// public void updateShoulderAngularVelocity() {
	// 	shoulderAngularVelocity = (lastShoulderAngle - getShoulderAngle()) / .001;
	// 	lastShoulderAngle = getShoulderAngle();
	// }

	// /**
	//  * Updates the angular velocity of the elbow -- must be called every 1ms
	//  */
	// public void updateElbowAngularVelocity() {
	// 	elbowAngularVelocity = (lastElbowAngle - getElbowAngle()) / .001;
	// 	lastElbowAngle = getElbowAngle();
	// }


	public void testMoveShoulder(double analogX) {
		double speedX = 12 * analogX; // 12V conversion
		// System.out.println(-0.3 * speedX + getShoulderFeedforward());
		shoulderMotor.setVoltage(-0.3 * speedX + getShoulderFeedforward());
	}

	public void testMoveElbow(double analogY) {
		double speedY = 12 * analogY; // 12V conversion
		elbowMotor.setVoltage(-0.3 * speedY + getElbowFeedforward());
	}

	/**
	 * Sets the end effector at the given (x, y) position using a control loop
	 * 
	 * @param xPos (double) x position of the end effector - METERS
	 * @param yPos (double) y position of the end effector - METERS
	 * @param theta (double) angle of the claw from the horizontal - DEGREES
	 */
	public double setEndEffector(double xPos, double yPos, double theta) {
		// TODO: maybe add something that sets the claw position to the param rather than handling it separately
		double[] angles = Kinematics.positionInverseKinematics(xPos, yPos, theta);
		
		double alpha = Math.toDegrees(angles[0]);
		double beta = Math.toDegrees(angles[1]);

		setShoulder(alpha);
		setElbow(beta);

		return angles[0];
	}

	public double[][] getTrajectory(double x, double y) {
		// USE WRIST JOINT POS SINCE IK CAN'T HANDLE WRIST YET
		double[] current_pos = Kinematics.forwardKinematicsWrist(getShoulderAngle(), getElbowAngle());

		double [][] trajectory = new LinearProfile().getSetPoints(
			new Vector(current_pos[0], current_pos[1]), 
			new Vector(x, y),
			getWristAngle()
		);

		return trajectory;
	}

	public double executeTrajectory(double[][] trajectory) {
		return 0;
	}

	//TODO: maybe use pid.setTolerance() to reduce oscillations from the chain?
	/**
	 * Uses the PID and feedforward control loops to set the shoulder at given setpoint
	 * 
	 * @param alpha (double) setpoint for the shoulder angle, in degrees
	 */
	public void setShoulder(double alpha) {
		// System.out.println(
		// 	shoulderFeedforward.calculate(alpha, 0, 0)
		// );
		// System.out.println(
		// 	shoulderPID.calculate(getShoulderAngle(), alpha)
		// );

		// neg sign because shoulder moves in the opposite direction
		shoulderMotor.setVoltage(
			-shoulderPID.calculate(getShoulderAngle(), alpha) + 
			getShoulderFeedforward()
		);
	}

	/**
	 * Uses the PID and feedforward control loops to set the elbow at given setpoint
	 * 
	 * @param beta (double) setpoint for the elbow angle, in degrees
	 */
	public void setElbow(double beta) {
		elbowMotor.setVoltage(
			-elbowPID.calculate(getElbowAngle(), beta) + 
			getElbowFeedforward()
		);
	}

	// TODO: copilot just spat this out but like maybe they just do the control loops for you??? goateedd
	// TODO: check units match with getShoulderAngularVelocity() and getElbowAngularVelocity()
	// private void setShoulderVelocity(double velocity) {
	// 	shoulderMotor.set(ControlMode.Velocity, velocity);
	// }
	
	// private void setElbowVelocity(double velocity) {
	// 	elbowMotor.set(ControlMode.Velocity, velocity);
	// }

	private void setShoulderVelocity(double voltage) {
		shoulderMotor.setVoltage(
			voltage + getShoulderFeedforward()
		);
	}
	
	private void setElbowVelocity(double voltage) {
		elbowMotor.setVoltage(
			voltage + getShoulderFeedforward()
		);
	}

	// Uses custom PID/Feedforward control loops - using the useless input so I can use overload and make it easier to switch between the two when testing
	//TODO: check units match with getShoulderAngularVelocity() and getElbowAngularVelocity()
	private void setShoulderVelocity(double velocity, double useless) {
		shoulderMotor.setVoltage(
			shoulderVelocityPID.calculate(getShoulderAngularVelocity(), velocity) +
			getShoulderFeedforward()
		);
	}
	
	private void setElbowVelocity(double velocity, double useless) {
		elbowMotor.setVoltage(
			elbowVelocityPID.calculate(getElbowAngularVelocity(), velocity) +
			getElbowFeedforward()
		);
	}

	/**
	 * This moves the arm at the given input speed in the horizontal and vertical directions
	 * 
	 * @param leftAnalogX (double) - [0, 1] controller X linear velocity input
	 * @param leftAnalogY (double) - [0, 1] controller Y linear velocity input
	 */
	public void moveArm(double leftAnalogX, double leftAnalogY) {
		double[] combinedSpeeds = Kinematics.speedInverseKinematics(leftAnalogX, leftAnalogY, getShoulderAngle(), getElbowAngle());

		// set the motor speeds as a percent 0-1 (normal) - leaving it commented out so I can test the velocity control
		// shoulderMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
		// elbowMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);

		// TODO: fix units - should be change encoder ticks per 100ms
		// setShoulderVelocity(combinedSpeeds[0] * Constants.SHOULDER_MAX_VELOCITY, 0);
		// setElbowVelocity(combinedSpeeds[1] * Constants.ELBOW_MAX_VELOCITY, 0);

		// setShoulderVelocity(combinedSpeeds[0] * 11);
		// setElbowVelocity(combinedSpeeds[0] * 11);

		// Uses PID loop to control arm with controller rather than setting a speed
		// setEndEffector(
		// 	Constants.MAX_FORWARD_X * leftAnalogX,
		// 	Constants.MAX_FORWARD_Y * leftAnalogY
		// );
	}

	// dont think we actually need this tbh but ill leave it in here in case we decide to use it
	public void setMotorLimits() {
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minAlpha + 2048);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxAlpha + 2048);
		

		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minBeta);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxBeta);
	}
}
