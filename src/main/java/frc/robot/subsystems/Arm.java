package frc.robot.subsystems;
import frc.robot.*;
import frc.robot.utils.*;

import java.io.*;
import java.util.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	WPI_TalonSRX shoulderMotorEncoder, elbowMotorEncoder;
	WPI_TalonFX elbowMotor, shoulderMotor;
	// TalonFXSensorCollection shoulderFalconSensor, elbowFalconSensor;

	// ProfiledPIDController shoulderPID, elbowPID;
	PIDController shoulderPID, elbowPID;
	// ArmFeedforward shoulderFeedforward, elbowFeedforward;
	// PIDController shoulderVelocityPID, elbowVelocityPID;

	DigitalInput limitSwitch;
	Grabber grabber;
	boolean grabberForward = true;

	double lastShoulderAngle, lastElbowAngle;
	double beforeStopShoulderAngle, beforeStopElbowAngle;
	double[] beforeStopJointAngles;
	double[] prevControllerPos;

	R2Vector linearVelocity = new R2Vector();
	R2Vector prevPosition = new R2Vector();

	// PID constants when tuning - TESTING ONLY
	public double shoulderP, shoulderI, shoulderD;
	public double elbowP, elbowI, elbowD;

	// Toggles whether motor limits are active - displayed on shuffleboard
	public boolean motorLimits = true;

	// Toggles whether the motors should stop - controlled manually
	public boolean manualMotorStop = false;

	int trajectoryCounter;
	ArrayList<double[]> trajectory;

	double[] targetAngles = new double[2];

	enum Mode {
		MANUAL, HOLDING_POSITION, RUNNING_TRAJECTORY, IDLE
	}
	Mode currentMode = Mode.IDLE;

	/**
	 * Constructor for Arm Class
	 * 
	 * @param shoulderTalonPort
	 * @param elbowTalonPort
	 */
	public Arm(int shoulderTalonPort, int shoulderFalconPort, int elbowTalonPort, int elbowFalconPort, DigitalInput limitSwitch, Grabber grabber) {
		// motor docs lol: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
		
		// initialize motors
        // shoulderMotorEncoder = new WPI_TalonSRX(shoulderTalonPort);
        // elbowMotorEncoder = new WPI_TalonSRX(elbowTalonPort);

		// initialize Falcon motors (USE LATER)
        shoulderMotor = new WPI_TalonFX(shoulderFalconPort);
        elbowMotor = new WPI_TalonFX(elbowFalconPort);
		

		// set the config to default in case there's something else I'm missing
		shoulderMotor.configFactoryDefault();
		elbowMotor.configFactoryDefault();

		// setup motor encoders TODO: fix it for falcons?
		TalonSRXConfiguration config = new TalonSRXConfiguration();
		config.peakCurrentLimit = 40; // the peak current, in amps
		config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
		config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
		// TODO: this isnt wokring with falcons
		// shoulderMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
		// elbowMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

		// https://github.com/GwhsRobotics3/Team-5507-2018/blob/b4d3e1d5e899132185e9f7b9711d5a92f322d659/src/org/usfirst/frc/team5507/robot/subsystems/DriveTrain.java#L112
		// not working with falcons
		// shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		// elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		this.limitSwitch = limitSwitch;
		this.grabber = grabber;

		shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);

		initControllers(false);
		init();

		// shoulderMotor.setNeutralMode(NeutralMode.Brake);
		// elbowMotor.setNeutralMode(NeutralMode.Brake);
    }

	public void init() {
		prevControllerPos = Kinematics.forwardKinematics(getShoulderAngle(), getWristAngle());
		lastShoulderAngle = getShoulderAngle(); 
		lastElbowAngle = getElbowAngle();

		beforeStopShoulderAngle = getShoulderAngle(); 
		beforeStopElbowAngle = getElbowAngle();

		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle(), getWristAngle());
		prevPosition = new R2Vector(pos[0], pos[1]);
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
		// shoulderVelocityPID = new PIDController(Constants.SHOULDER_VELOCITY_KP, Constants.SHOULDER_VELOCITY_KI, Constants.SHOULDER_VELOCITY_KD);
        // elbowVelocityPID = new PIDController(Constants.ELBOW_VELOCITY_KP, Constants.ELBOW_VELOCITY_KI, Constants.ELBOW_VELOCITY_KD);
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
		// shoulderVelocityPID = new PIDController(Constants.SHOULDER_VELOCITY_KP, Constants.SHOULDER_VELOCITY_KI, Constants.SHOULDER_VELOCITY_KD);
        // elbowVelocityPID = new PIDController(Constants.ELBOW_VELOCITY_KP, Constants.ELBOW_VELOCITY_KI, Constants.ELBOW_VELOCITY_KD);
	}

	///////////////
	// MAIN LOOP //
	///////////////

	/**
	 * Basically just does everything the arm needs to do on its own every 20ms loop
	 */
	public void run(double leftAnalogX, double leftAnalogY, double rightAnalogX, double rightAnalogY) {
		if (leftAnalogX !=0 || leftAnalogY != 0 || rightAnalogX != 0 || rightAnalogY != 0) {
			stopTrajectory();
		}

		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());
		double x = pos[0]; double y = pos[1];

		setWrist(wristDesiredPosition(x, y));

		switch(currentMode) {
			case IDLE:
				break;
			case MANUAL:
				// if (shouldMotorStop()) {
					// stopMotors();
				// 	break;
				// }
				if (leftAnalogX == 0 && leftAnalogY == 0) {
					moveShoulder(rightAnalogX);
					moveElbow(rightAnalogY);
				} else {
					// TODO: add controls for linear movement
					// moveArm(leftAnalogX, leftAnalogY);
				}
				break;

			case HOLDING_POSITION:
				double alpha = targetAngles[0];
				double beta = targetAngles[1];

				setShoulder(alpha);
				setElbow(beta);
				break;

			case RUNNING_TRAJECTORY:
				executeTrajectory();
				break;
		}
	}

	public void run() {
		run(0, 0, 0, 0);
	}

	//////////////////////
	// VARIABLE TOGGLES //
	//////////////////////
	
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

	/////////////
	// UPDATES //
	/////////////

	public double[] updateBeforeStopAngles() {
		if (!Kinematics.shouldMotorStop(getShoulderAngle(), getElbowAngle(), getWristAngle())) { 
			beforeStopShoulderAngle = getShoulderAngle();
			beforeStopElbowAngle = getElbowAngle();
			beforeStopJointAngles = getJointAngles(); 
		}
		SmartDashboard.putNumber("LAST SHOULDER", beforeStopShoulderAngle);
		SmartDashboard.putNumber("LAST ELBOW", beforeStopElbowAngle);

		return beforeStopJointAngles;
	}

	public R2Vector updateLinearVelocity() {
		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle(), getWristAngle());
		R2Vector currentPosition = new R2Vector(pos[0], pos[1]);
		linearVelocity = currentPosition.sub(prevPosition).multiply(1.0 / 0.02); 
		return linearVelocity;
	}

	/////////////
	// GETTERS //
	/////////////

	public boolean shouldMotorStop() {
		double alpha = getShoulderAngle();
		double beta = getElbowAngle();
		double theta = getWristAngle();

		boolean motorStop = Kinematics.shouldMotorStop(alpha, beta, theta);

		return (motorStop && motorLimits) || manualMotorStop; // check if the motor limits are activated or if driver is trying to stop them manually
	}

	public double getShoulderRawAngle() { 
		return shoulderMotor.getSelectedSensorPosition();
	}

	public double wristDesiredPosition(double x, double y) {
		if (Kinematics.wristDesiredPosition(x, y) == 90 && grabberForward) {
			return 90;
		} if (Kinematics.wristDesiredPosition(x, y) == 0 && !grabberForward) {
			return 0;
		}

		return getWristAngle();
	}

	/**
	 * @return angle (double) degrees of the first linkage from the horizontal
	 */
	public double getShoulderAngle() {
		// experimentally found ratio TODO: fix for falcons
		return -shoulderMotor.getSelectedSensorPosition() / 1137.7  + 5.7;
		// return shoulderEncoder.getSelectedSensorPosition();
	}

	public double getElbowAngle() {
		SmartDashboard.putNumber("FALCON ELBOW RAW: ", elbowMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("FALCON ELBOW: ", elbowMotor.getIntegralAccumulator() * 360/4096 / 200);
		return -(elbowMotor.getSelectedSensorPosition() / 1137.7 + 84.5);
	}

	/**
	 * @return angle (double) degrees of the second linkage from the horizontal
	 */
	public double getElbowOldAngle() {
		// encoder reads in [-4096, 0], and absolute position is off by 10 degrees 
		// offset  by shoulder angle so that the angle is relative to the horizotal
		double angle = getShoulderAngle() - ((elbowMotorEncoder.getSelectedSensorPosition(1) + 1300) * 360/4096) + 14;
		return angle;
	}

	public double[] getJointAngles() {
		return new double[] {getShoulderAngle(), getElbowAngle()};
	}

	/**
	 * @return angle (double) degrees of the third linkage (claw) from the horizontal
	 */
	public double getWristAngle() {
		int wristActuated = grabber.isWristActuated() ? 0:1;
		return getShoulderAngle() + getElbowAngle() + wristActuated * 90;
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

		/**
	 * Returns the voltage needed to counteract gravity
	 * 
	 * @return controlInput (double) voltage to send to motors
	 */
	public double getElbowFeedforward() {
		double gravity = Constants.ELBOW_KG * Math.cos(Math.toRadians(getShoulderAngle() + getElbowAngle()));
		double velocity = Constants.ELBOW_KV * getElbowAngularVelocity();
		return - (gravity);
	}


	//TODO: FUNCTION RETURNS A CONSTANTS VALUE LOL SOMEONE NEEDS TO FIX THAT
	/**
	 * Calculates the COM of the arm and returns the voltage needed to counteract gravity
	 * Diagram: https://raw.githubusercontent.com/CCHS-FIRST-Robotics/2023DriveBase/main/images/B6D091A1-433E-4F3A-8475-E74F224E33DC.png
	 * 
	 * @return controlInput (double) voltage to send to motors 
	 */
	public double getShoulderFeedforward() {
		double d1 = Constants.LOWER_ARM_COM_DIST;
		double d2 = Constants.UPPER_ARM_COM_DIST;
		double m1 = Constants.LOWER_ARM_MASS;
		double m2 = Constants.UPPER_ARM_MASS;
		double l1 = Constants.LOWER_ARM_LENGTH;

		double alpha = Math.toRadians(getShoulderAngle());
		double beta = alpha - Math.toRadians(getElbowAngle());

		double comX = (m1*d1*Math.cos(alpha) + m2*(l1*Math.cos(alpha) + d2*Math.cos(alpha + beta))) / (m1 + m2);
		double comY = (m1*d1*Math.sin(alpha) + m2*(l1*Math.sin(alpha) + d2*Math.sin(alpha + beta))) / (m1 + m2);
		
		// TODO: using atan2 might fix the sign error, have to test - fuck it lets just try it
		double controlInput = -Constants.SHOULDER_KG * Math.cos(Math.atan2(comY, comX));
		return controlInput;
		// // atan returns between -pi/2 and pi/2, but cos only returns pos values on that interval, so we have to check the sign manually
		// if (alpha > Math.PI / 2) {
		// 	return -controlInput;
		// } else {
		// 	return controlInput;
		// }
	}

	public Mode getCurrentMode() {
		return currentMode;
	}

	public ArrayList<double[]> getTrajectory(double x, double y) {
		// USE WRIST JOINT POS SINCE IK CAN'T HANDLE WRIST YET
		double[] current_pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());

		ArrayList<double[]> trajectory = new QuadraticProfile().getSetPoints(
			new R2Vector(current_pos[0], current_pos[1]),  // initial pos
			getJointAngles(), // initial angles
			new R2Vector(x, y), // goal pos
			getWristAngle(), // wrist angle
			Constants.ARM_MAX_SPEED,
			Constants.ARM_MAX_ACCELERATION
		);

		return trajectory;
	}

	//////////////////////
	// SETTERS/MOVEMENT //
	//////////////////////

	/**
	 * Sets the motor outputs to resist gravity and sets the motors to brake mode
	 */
	public void stopMotors() {
		// System.out.println(getElbowFeedforward());
		
		// shoulderMotor.setVoltage(getShoulderFeedforward());
		// elbowMotor.setVoltage(getElbowFeedforward());
		switch(currentMode) {
			case HOLDING_POSITION:
				return;
			case RUNNING_TRAJECTORY:
				return;
		}

		setShoulder(beforeStopShoulderAngle);
		setElbow(beforeStopElbowAngle);

		shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);
	}

	public void moveShoulder(double analogX) {
		if (analogX == 0) {
			setShoulder(lastShoulderAngle);
			return;
		}
		lastShoulderAngle = getShoulderAngle();
		double speedX = 12 * analogX; // 12V conversion
		// System.out.println(-0.3 * speedX + getShoulderFeedforward());
		shoulderMotor.setVoltage(-0.3 * speedX + getShoulderFeedforward());
	}

	public void moveElbow(double analogY) {
		if (analogY == 0) {
			setElbow(lastElbowAngle);
			return;
		}
		lastElbowAngle = getElbowAngle();
		double speedY = 12 * analogY; // 12V conversion
		elbowMotor.setVoltage(-0.3 * speedY + getElbowFeedforward());
	}

	public void setNeutralPostion() {
		grabber.clawBack(); // open claw
		grabber.wristForward(); // wrist in line with upper arm

		setEndEffector(1, 1, 0);
	}

	public void setEndEffector(double xPos, double yPos, double theta) {
		setEndEffector(xPos, yPos, theta, true);
	}

	/**
	 * Sets the end effector at the given (x, y) position using a control loop
	 * 
	 * @param xPos (double) x position of the end effector - METERS
	 * @param yPos (double) y position of the end effector - METERS
	 * @param theta (double) angle of the claw from the horizontal - DEGREES
	 */
	public void setEndEffector(double xPos, double yPos, double theta, boolean debug) {
		// TODO: maybe add something that sets the claw position to the param rather than handling it separately
		// double[] angles = Kinematics.positionInverseKinematics(xPos, yPos, getJointAngles());

		trajectory = Kinematics.degrees(getTrajectory(xPos, yPos));
		// when goes past motor limit traj is empty
		if (trajectory.size() < 1) {
			return;
		}
		currentMode = Mode.RUNNING_TRAJECTORY;
		trajectoryCounter = 0;
		executeTrajectory();

		if (debug) printTrajInfo(trajectory, xPos, yPos);
	}

	public void executeTrajectory() {
		targetAngles = trajectory.get(trajectoryCounter);
		trajectoryCounter++;

		setElbow(targetAngles[1]);
		setShoulder(targetAngles[0]);

		if (trajectoryCounter >= trajectory.size()) {
			trajectoryCounter = 0;
			currentMode = Mode.HOLDING_POSITION;
		} 
	}

	public void stopTrajectory() {
		trajectoryCounter = 0;
		currentMode = Mode.MANUAL;
	}

	public void printTrajInfo(ArrayList<double[]> trajectory, double x, double y) {
		// when goes past motor limit traj is empty
		if (trajectory.size() < 1) {
			return;
		};
		System.out.println("GOT HERE 1");
	
		for (int i=0; i<trajectory.size(); i++) {
		  double[] angles = trajectory.get(i);
		  System.out.println("Angles: " + angles[0] + " next " + angles[1]);
		}
		double[] angles = trajectory.get(trajectory.size() - 1);
		System.out.println("GOT HERE 2");
		System.out.println("LAST: " + angles[0] + " next " + angles[1]);
	
		System.out.println("IK SOLUTION X: " + Math.toDegrees(Kinematics.positionInverseKinematics(x, y, getJointAngles()) [0]));
		System.out.println("IK SOLUTION Y: " + Math.toDegrees(Kinematics.positionInverseKinematics(x, y, getJointAngles()) [1]));
	
		System.out.println("FK TEST X: " + Kinematics.forwardKinematics(angles[0], angles[1]) [0]);
		System.out.println("FK TEST Y: " + Kinematics.forwardKinematics(angles[0], angles[1]) [1]);
	  }

	//TODO: maybe use pid.setTolerance() to reduce oscillations from the chain?
	/**
	 * Uses the PID and feedforward control loops to set the shoulder at given setpoint
	 * 
	 * @param alpha (double) setpoint for the shoulder angle, in degrees
	 */
	public void setShoulder(double alpha) {
		// System.out.println(
		// 	getShoulderFeedforward()
		// );
		// System.out.println(
		// 	shoulderPID.calculate(getShoulderAngle(), alpha)
		// );

		SmartDashboard.putNumber("DESIRED ALPHA", alpha);

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

		SmartDashboard.putNumber("DESIRED BETA", beta);

		elbowMotor.setVoltage(
			-elbowPID.calculate(getElbowAngle(), beta) + 
			getElbowFeedforward() 
			
		);
	}

	public void setWrist(double theta) {
		if (theta == 90) {
			grabber.wristBack();
		} if (theta == 0) {
			grabber.wristForward();
		}
	}

	/**
	 * This moves the arm at the given input speed in the horizontal and vertical directions
	 * 
	 * @param leftAnalogX (double) - [0, 1] controller X linear velocity input
	 * @param leftAnalogY (double) - [0, 1] controller Y linear velocity input
	 */
	public void moveArm(double leftAnalogX, double leftAnalogY) {
		// double[] combinedSpeeds = Kinematics.speedInverseKinematics(leftAnalogX, leftAnalogY, getShoulderAngle(), getElbowAngle());

		// set the motor speeds as a percent 0-1 (normal) - leaving it commented out so I can test the velocity control
		// shoulderMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
		// elbowMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);

		// TODO: fix units - should be change encoder ticks per 100ms
		// setShoulderVelocity(combinedSpeeds[0] * Constants.SHOULDER_MAX_VELOCITY, 0);
		// setElbowVelocity(combinedSpeeds[1] * Constants.ELBOW_MAX_VELOCITY, 0);

		// setShoulderVelocity(combinedSpeeds[0] * 11);
		// setElbowVelocity(combinedSpeeds[0] * 11);

		// Uses PID loop to control arm with controller rather than setting a speed
		double speedX = Constants.MAX_FORWARD_X * -leftAnalogX;
		double speedY = Constants.MAX_FORWARD_Y * leftAnalogY;
		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());

		// If we're moving, keep updating the position, otherwise keep what we started at
		// Prevents the y pos from increasing when youre only trying to move in the x
		if (speedX != 0) {
			prevControllerPos[0] = pos[0];
		}
		if (speedY != 0) {
			prevControllerPos[1] = pos[1];
		}

		// System.out.println("DESIRED X: " + Math.min(prevControllerPos[0] + speedX, Constants.maxX - .05));
		// System.out.println("DESIRED Y: " + Math.min(prevControllerPos[1] + speedY, Constants.maxY - .05));
		setEndEffector(
			Math.min(prevControllerPos[0] + speedX, Constants.maxX - .05),
			Math.min(prevControllerPos[1] + speedY, Constants.maxY - .05),
			getWristAngle()
		);
	}


	////////////////
	// UNUSED ATM //
	////////////////

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

	// dont think we actually need this tbh but ill leave it in here in case we decide to use it
	public void setMotorLimits() {
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minAlpha + 2048);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxAlpha + 2048);
		

		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minBeta);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxBeta);
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
	// private void setShoulderVelocity(double velocity, double useless) {
	// 	shoulderMotor.setVoltage(
	// 		shoulderVelocityPID.calculate(getShoulderAngularVelocity(), velocity) +
	// 		getShoulderFeedforward()
	// 	);
	// }
	
	// private void setElbowVelocity(double velocity, double useless) {
	// 	elbowMotor.setVoltage(
	// 		elbowVelocityPID.calculate(getElbowAngularVelocity(), velocity) +
	// 		getElbowFeedforward()
	// 	);
	// }
}
