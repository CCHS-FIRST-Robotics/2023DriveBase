package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;


public class Arm {

    // define motor and encoder objects
	WPI_TalonSRX shoulderMotor, elbowMotor;
	// TalonFXSensorCollection shoulderFalconSensor, elbowFalconSensor;


	//TODO: add claw encoder and PID constants
	PIDController shoulderPID;
	PIDController elbowPID;

    // current constant to be tuned
	int currentPIDConstant = 0; // 0 = kP, 1 = kI, 2 = kD

	// during PID Tuning Mode, either increasing or decreasing the constants by the increment
	boolean increasingPIDConstant = true;

    /**
	 * Constructor for Arm Class -- setup as tankdrive until I figure out what motors we're using 
	 * 
	 * @param shoulderTalonPort
	 * @param elbowTalonPort
	 */
	public Arm(int shoulderTalonPort, int elbowTalonPort) {
		// motor docs lol: https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
		// initialize motors
        shoulderMotor = new WPI_TalonSRX(shoulderTalonPort);
        elbowMotor = new WPI_TalonSRX(elbowTalonPort);

		// setup motor encoders
		TalonSRXConfiguration config = new TalonSRXConfiguration();
		config.peakCurrentLimit = 40; // the peak current, in amps
		config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
		config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
		shoulderMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
		elbowMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

		// initialize Falcon motors (USE LATER)
        // shoulderMotor = new WPI_TalonFX(shoulderTalonPort);
        // elbowMotor = new WPI_TalonFX(elbowTalonPort);

		// Initializing Falcon sensors (USE LATER)
		//shoulderFalconSensor = new TalonFXSensorCollection(shoulderMotor);
		//elbowFalconSensor = new TalonFXSensorCollection(elbowMotor);

        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD);
    }

	public double getShoulderAngle() {
		return shoulderMotor.getSelectedSensorPosition(); // prints the position of the selected sensor
		// return shoulderFalconSensor.getIntegratedSensorAbsolutePosition();
	}

	public double getElbowAngle() {
		return elbowMotor.getSelectedSensorPosition();
		// return elbowFalconSensor.getIntegratedSensorAbsolutePosition();
	}

	public void setShoulder() {
	}

	public void setElbow() {
	}

	public void moveArm(double leftAnalogX, double leftAnalogY) {
		double[] combinedSpeeds = speedInverseKinematics(leftAnalogX, leftAnalogY);

		// set the motor speeds as a percent 0-1 (normal)
		shoulderMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
		elbowMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
	}

	public void testMove() {
		// elbowMotor.set(ControlMode.PercentOutput, -.2);
		shoulderMotor.set(ControlMode.PercentOutput, .2);
	}

	//TODO: write method
    /**
	 * This will return the desired x, y pos for the given angle of each length
	 * 
	 * @param xPos (double)
     * @param yPos (double)
	 * @return angles
	 */
    private double[] forwardKinematics(double alpha, double beta) {
		double[] x = {0, 0};
        return x;
    }

    /**
	 * This will return the desired angle of each arm length using IK
	 * Calculated by finding the (x, y) coords of the center joint by using 
	 * the intersection of the two circles at the joint and end effector
	 * with radius equal to the arm segement length
	 * Intersection solution from: https://math.stackexchange.com/a/1033561
	 * 
	 * @param xPos (double)
     * @param yPos (double)
	 * @return angles (double[2]) degrees
	 */
    private double[] positionInverseKinematics(double xPos, double yPos) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;


		double dist = Math.sqrt(xPos*xPos + yPos*yPos);
		double l = l1*l1 - l2*l2 + dist*dist;
		double h = Math.sqrt(l1*l1 - l*l);

		// double x1 = l/dist * xPos - h/dist * yPos;
    	double y1 = l/dist * yPos + h/dist * xPos;

		double alpha = Math.asin(y1/l1);
    	double beta = -Math.PI/2 + alpha - Math.asin((l1*l1 + l2*l2 - dist*dist)/(2*l1*l2));
		
		double[] angles = {alpha, beta};
        return angles;
    }

	/**
	 * This will return the desired speed of each motor using IK
	 * 
	 * @param xSpeed (double) percent [0, 1]
     * @param ySpeed (double) percent [0, 1]
	 * @return speeds (double[2]) motor speeds - percent [0, 1]
	 */
    private double[] speedInverseKinematics(double xSpeed, double ySpeed) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;

		double alpha = getAlpha();
		double beta = getBeta();

		double v1x = l1 * Math.cos(alpha);
		double v1y = l1 * Math.sin(alpha);

		double v2x = l2 * Math.cos(beta - alpha);
		double v2y = l2 * Math.sin(beta - alpha);

		double w1x, w2x, w1y, w2y;

		// LATERAL MOVEMENT:
		if (v2y > v1y) {
			w1x = 1;
			w2x = -v1y/v2y;
		} else {
			w2x = 1;
			w1x = -v2y/v1y;
		}

		// VERTICAL MOVEMENT:
		if (v2x > v1x) {
			w1y = 1;
			w2y = -v1x/v2x;
		} else {
			w2y = 1;
			w1y = -v2x/v1x;
		}

		double[] combinedSpeeds = {w1x + w1y, w2x + w2y};

        return combinedSpeeds;
    }
}
