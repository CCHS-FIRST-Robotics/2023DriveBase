package frc.robot.subsystems;
import frc.robot.*;
import frc.robot.Constants.ArmFixedPosition;
import frc.robot.utils.*;

import java.io.*;
import java.util.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
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

	// ProfiledPIDController shoulderPID, elbowPID;
	PIDController shoulderPID, elbowPID;

	DigitalInput limitSwitch;
	Grabber grabber;
	boolean grabberForward = true;

	double lastShoulderAngle, lastElbowAngle;
	double beforeStopShoulderAngle, beforeStopElbowAngle;
	double[] beforeStopJointAngles;
	double[] prevControllerPos;

	R2Vector linearVelocity = new R2Vector();
	R2Vector prevPosition = new R2Vector();

	double currentPositionX;
	double currentPositionY;

	double speedMultipler = 1;

	// PID constants when tuning - TESTING ONLY
	public double shoulderP, shoulderI, shoulderD;
	public double elbowP, elbowI, elbowD;

	// Toggles whether motor limits are active - displayed on shuffleboard
	public boolean motorLimits = true;

	// Toggles whether the motors should stop - controlled manually
	public boolean manualMotorStop = false;

	boolean wristForced = false;

	int trajectoryCounter = 0;
	ArrayList<double[]> trajectory;

	double[] targetAngles = new double[2];

	public enum Mode {
		MANUAL, 
		HOLDING_POSITION, 
		RUNNING_TRAJECTORY,
		IDLE
	}
	
	public Mode currentMode = Mode.IDLE;

	/**
	 * Constructor for Arm Class
	 * 
	 * @param shoulderTalonPort
	 * @param elbowTalonPort
	 */
	public Arm(int shoulderTalonPort, int shoulderFalconPort, int elbowTalonPort, int elbowFalconPort, DigitalInput limitSwitch, Grabber grabber) {
		// motor docs lol: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
		
		// initialize motors
        shoulderMotorEncoder = new WPI_TalonSRX(shoulderTalonPort);
        elbowMotorEncoder = new WPI_TalonSRX(elbowTalonPort);

		// initialize Falcon motors (USE LATER)
        shoulderMotor = new WPI_TalonFX(shoulderFalconPort);
        elbowMotor = new WPI_TalonFX(elbowFalconPort);
		
		this.limitSwitch = limitSwitch;
		this.grabber = grabber;

		
		// set motor configs
		configTalonFX(shoulderMotor);
		configTalonFX(elbowMotor);
		
		// shoulderMotor.setNeutralMode(NeutralMode.Coast);
		// elbowMotor.setNeutralMode(NeutralMode.Coast);

		prevControllerPos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());
		currentPositionX = prevControllerPos[0];
		currentPositionY = prevControllerPos[1];

		lastShoulderAngle = getShoulderAngle(); 
		lastElbowAngle = getElbowAngle();

		beforeStopShoulderAngle = lastShoulderAngle; 
		beforeStopElbowAngle = lastElbowAngle;
		targetAngles[0] = lastShoulderAngle;
		targetAngles[1] = lastElbowAngle;


		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle(), getWristAngle());
		prevPosition = new R2Vector(pos[0], pos[1]);

		// Positional PID Controllers
        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD);

		shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);
    }

	public void configTalonFX(WPI_TalonFX talon) {
		talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
										   Constants.FALCON_PID_IDX, 
										   Constants.FALCON_TIMEOUT_MS);
		// talon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		// talon.getSensorCollection().setIntegratedSensorPositionToAbsolute(Constants.FALCON_TIMEOUT_MS);
		
		talon.configNominalOutputForward(0, Constants.FALCON_TIMEOUT_MS);
		talon.configNominalOutputReverse(0, Constants.FALCON_TIMEOUT_MS);
		talon.configPeakOutputForward(1, Constants.FALCON_TIMEOUT_MS);
		talon.configPeakOutputReverse(-1, Constants.FALCON_TIMEOUT_MS);

		talon.configClosedLoopPeakOutput(Constants.FALCON_PID_IDX, 0.5);
	}

	///////////////
	// MAIN LOOP //
	///////////////

	/**
	 * Basically just does everything the arm needs to do on its own every 20ms loop
	 */
	public void run(double leftAnalogX, double leftAnalogY, double rightAnalogX, double rightAnalogY) {
		double[] pos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());
		double x = pos[0]; 
		double y = pos[1];
		
		// when a joystick is moved stop movement and switch to manual mode
		if (!Constants.isZero(leftAnalogX) || 
		!Constants.isZero(leftAnalogY)|| 
		!Constants.isZero(rightAnalogX) || 
		!Constants.isZero(rightAnalogY)) {
			stopTrajectory();
			wristForced = false;
			prevControllerPos = Kinematics.forwardKinematics(getShoulderAngle(), getElbowAngle());
		} else {
			// keep track of position when not moving joystick
			currentPositionX = x; 
			currentPositionY = y;
			
			if (currentMode == Mode.MANUAL) {
				currentMode = Mode.HOLDING_POSITION;

				double prevX = prevControllerPos[0];
				double prevY = prevControllerPos[1];
				targetAngles = Kinematics.degrees(Kinematics.positionInverseKinematics(prevX, prevY, true));
			}
		}

		// System.out.println(wristDesiredPosition(x, y));
		// setWrist(wristDesiredPosition(x, y));

		switch(currentMode) {
			case IDLE: break;
			case MANUAL:
				// if (shouldMotorStop()) {
					// stopMotors();
				// 	break;
				// }
				// if (Constants.isZero(leftAnalogX) 
				// 	&& Constants.isZero(leftAnalogY)
				// ) {
				// 	// moveShoulder(rightAnalogX);
				// 	// moveElbow(rightAnalogY);
				// } else {
				// 	// TODO: add controls for linear movement
				// 	moveArm(leftAnalogX, leftAnalogY);
				// }
				moveArm(leftAnalogX, leftAnalogY);
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

	public double getSpeedMultipler() {
		return speedMultipler;
	}

	public void setSpeedMultipler(double speedMultipler) {
		this.speedMultipler = speedMultipler;
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
		System.out.println(Kinematics.wristDesiredPosition(x, y)); 
		System.out.println(grabberForward);
		System.out.println(!wristForced);
		if (Kinematics.wristDesiredPosition(x, y) == 1 && grabberForward && !wristForced) {
			return 1;
		} if (Constants.isZero(Kinematics.wristDesiredPosition(x, y)) && !grabberForward && !wristForced) {
			return 0;
		}

		return -1;
	}

	/**
	 * @return angle (double) degrees of the first linkage from the horizontal
	 */
	public double getShoulderAngle() {
		return -shoulderMotor.getSelectedSensorPosition() / 1137.7  + 95.5;
	}

	public double getElbowAngle() {
		return -(elbowMotor.getSelectedSensorPosition() / 1137.7 + 164.5);
	}

	// public double getShoulderAngle() {
	// 	// return shoulderMotorEncoder.getSelectedSensorPosition();
	// 	return -shoulderMotorEncoder.getSelectedSensorPosition() * 360 / 4096 + 75;
	// }

	// public double getElbowAngle() {
	// 	return -(elbowMotorEncoder.getSelectedSensorPosition() * 360 / 4096 + 165.5);
	// }

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
	 * Returns the voltage needed to counteract gravity
	 * 
	 * @return controlInput (double) voltage to send to motors
	 */
	public double getElbowFeedforward() {
		double gravity = Constants.ELBOW_KG * Math.cos(Math.toRadians(getShoulderAngle() + getElbowAngle()));
		return -gravity;
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
		
		return -Constants.SHOULDER_KG * Math.cos(Math.atan2(comY, comX));
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

		// shoulderMotor.setNeutralMode(NeutralMode.Coast);
		// elbowMotor.setNeutralMode(NeutralMode.Coast);
	}

	public void moveShoulder(double analogX) {
		if (Constants.isZero(analogX)) {
			// setShoulder(lastShoulderAngle);
			// return;
		}
		lastShoulderAngle = getShoulderAngle();
		double speedX = 12 * analogX; // 12V conversion
		// System.out.println(-0.3 * speedX + getShoulderFeedforward());
		shoulderMotor.setVoltage(-0.3 * speedX + getShoulderFeedforward());
	}

	public void moveElbow(double analogY) {
		if (Constants.isZero(analogY)) {
			// setElbow(lastElbowAngle);
			// return;
		}
		lastElbowAngle = getElbowAngle();
		double speedY = 12 * analogY; // 12V conversion
		elbowMotor.setVoltage(-0.3 * speedY + getElbowFeedforward());
	}

	public void setNeutralPostion() {
		grabber.clawForward(); // close claw
		setWrist(0); // wrist in line with upper arm

		setEndEffector(Constants.ArmFixedPosition.NEUTRAL);
	}

	public void setEndEffector(double xPos, double yPos) {
		setEndEffector(xPos, yPos, false);
	}

	public void setEndEffector(ArmFixedPosition position) {
		double x, y;
		wristForced = false;
		switch (position) {
			case CUBE_LOWER:
				x = Constants.CUBE_LOWER.x;
				y = Constants.CUBE_LOWER.y;
				break;
			case CUBE_HIGHER:
				x = Constants.CUBE_HIGHER.x;
				y = Constants.CUBE_HIGHER.y;
				break;
			case CONE_LOWER:
				x = Constants.CONE_LOWER.x;
				y = Constants.CONE_LOWER.y;
				break;
			case CONE_HIGHER_PRE_POS:
				x = Constants.CONE_HIGHER_PRE_POS.x;
				y = Constants.CONE_HIGHER_PRE_POS.y;
				System.out.println("POS: "+ x +" next " + y);
				break;
			case CONE_HIGHER:
				x = Constants.CONE_HIGHER.x;
				y = Constants.CONE_HIGHER.y;
				break;
			case DROPOFF_LOW:
				x = Constants.DROPOFF_LOW.x;
				y = Constants.DROPOFF_LOW.y;
				break;
			case PICKUP_GROUND:
				setWrist(90);
				grabber.clawBack();
				wristForced = true;
				x = Constants.PICKUP_GROUND.x;
				y = Constants.PICKUP_GROUND.y;
				break;
			case PICKUP_GROUND_LAYING_DOWN:
				setWrist(0);
				grabber.clawBack();
				wristForced = true;
				x = Constants.PICKUP_GROUND_LAYING_DOWN.x;
				y = Constants.PICKUP_GROUND_LAYING_DOWN.y;
				break;
			case PICKUP_SUBSTATION:
				grabber.clawBack();
				x = Constants.PICKUP_SUBSTATION.x;
				y = Constants.PICKUP_SUBSTATION.y;
				break;
			case NEUTRAL:
				setWrist(0);
				wristForced = true;
				x = Constants.NEUTRAL.x;
				y = Constants.NEUTRAL.y;
				break;
			default:
				return;
		}
		setEndEffector(x, y);
	}

	/**
	 * Sets the end effector at the given (x, y) position using a control loop
	 * 
	 * @param xPos (double) x position of the end effector - METERS
	 * @param yPos (double) y position of the end effector - METERS
	 * @param theta (double) angle of the claw from the horizontal - DEGREES
	 */
	public void setEndEffector(double xPos, double yPos, boolean debug) {
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
		
		setShoulder(targetAngles[0]);
		setElbow(targetAngles[1]);

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

	/**
	 * Uses the PID and feedforward control loops to set the shoulder at given setpoint
	 * 
	 * @param alpha (double) setpoint for the shoulder angle, in degrees
	 */
	public void setShoulder(double alpha) {
		// TODO: move to shuffleboard
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
			-elbowPID.calculate(getElbowAngle(), beta) + getElbowFeedforward()
		);
	}

	public void setWrist(double theta) {
		if (theta == 1) {
			grabber.wristBack();
			grabberForward = false;
		} if (Constants.isZero(theta)) {
			grabber.wristForward();
			grabberForward = true;
		}
	}

	/**
	 * This moves the arm at the given input speed in the horizontal and vertical directions
	 * 
	 * @param x (double) - [-1, 1] controller X linear velocity input
	 * @param y (double) - [-1, 1] controller Y linear velocity input
	 */
	public void moveArm(double x, double y) 
	{
		// MAX_FORWARD_X, MAX_FORWARD_Y distance to be traveled in 20ms
		double newposX = currentPositionX + Constants.MAX_FORWARD_X * speedMultipler * x;
		double newposY = currentPositionY + Constants.MAX_FORWARD_Y * speedMultipler * y;

		double wristAngle = getWristAngle();
		double wristIncX = Math.cos(Math.toRadians(wristAngle)) * Constants.WRIST_LENGTH;
		double wristIncY = Math.sin(Math.toRadians(wristAngle)) * Constants.WRIST_LENGTH;

		double directionX = newposX - currentPositionX;
		double directionY = newposY - currentPositionY;

		// System.out.println("DIR: " + directionX);

		if ((newposX + wristIncX > Constants.minX || directionX > 0) &&
			(newposX + wristIncX < Constants.maxX || directionX < 0) &&
			Kinematics.isPositionPossible(newposX, newposY))
		{
			currentPositionX = newposX;
		}

		if ((newposY + wristIncY > Constants.minY || directionY > 0) &&
			(newposY + wristIncY < Constants.maxY || directionY < 0) &&
			Kinematics.isPositionPossible(newposX, newposY))
		{
			currentPositionY = newposY;
		}

		double[] angles = Kinematics.degrees(Kinematics.positionInverseKinematics(currentPositionX, currentPositionY, true));
		// System.out.println("x: " + currentPositionX);
		// System.out.println("y: " + currentPositionY);
		// System.out.println("ALPHA: " + angles[0]);
		// System.out.println("BETA: " + angles[1]);
		
		setShoulder(angles[0]);
		setElbow(angles[1]);

	}
}