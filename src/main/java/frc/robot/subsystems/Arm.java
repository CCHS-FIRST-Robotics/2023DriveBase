package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;


public class Arm {

    // define motor and encoder objects
	WPI_TalonSRX shoulderMotor, elbowMotor;
	// TalonFXSensorCollection shoulderFalconSensor, elbowFalconSensor;


	//TODO: add claw encoder and PID constants
	PIDController shoulderPID, elbowPID;
	ArmFeedforward shoulderFeedforward, elbowFeedforward;

    // current constant to be tuned
	int currentPIDConstant = 0; 

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

		// set the config to default in case there's something else I'm missing
		shoulderMotor.configFactoryDefault();
		elbowMotor.configFactoryDefault();

		// setup motor encoders
		TalonSRXConfiguration config = new TalonSRXConfiguration();
		config.peakCurrentLimit = 40; // the peak current, in amps
		config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
		config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
		shoulderMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
		elbowMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

		// https://github.com/GwhsRobotics3/Team-5507-2018/blob/b4d3e1d5e899132185e9f7b9711d5a92f322d659/src/org/usfirst/frc/team5507/robot/subsystems/DriveTrain.java#L112
		shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);


		// initialize Falcon motors (USE LATER)
        // shoulderMotor = new WPI_TalonFX(shoulderTalonPort);
        // elbowMotor = new WPI_TalonFX(elbowTalonPort);

		// Initializing Falcon sensors (USE LATER)
		//shoulderFalconSensor = new TalonFXSensorCollection(shoulderMotor);
		//elbowFalconSensor = new TalonFXSensorCollection(elbowMotor);

        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD);

		shoulderFeedforward = new ArmFeedforward(Constants.SHOULDER_KS, Constants.SHOULDER_KG, Constants.SHOULDER_KV, Constants.SHOULDER_KA);
		elbowFeedforward = new ArmFeedforward(Constants.ELBOW_KS, Constants.ELBOW_KG, Constants.ELBOW_KV, Constants.ELBOW_KA);
    }

	public boolean shouldMotorStop() {
		return (
			getShoulderAngle() < Constants.minAlpha |
			getShoulderAngle() > Constants.maxAlpha |
			getElbowAngle() < Constants.minBeta | 
			getElbowAngle() > Constants.maxBeta
		);
	}

	public void setMotorLimits() {
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * -Constants.minAlpha + 2048);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxAlpha + 2048);
		

		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * -Constants.minBeta);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxBeta);
	}

	/**
	 * @return angle (double) degrees of the first linkage from the horizontal
	 */
	public double getShoulderAngle() {
		// encoder reads in [-2048, 2048] god knows why it's not the same as the other
		return 360 - (shoulderMotor.getSelectedSensorPosition(1) + 2048) * 360/4096 - 95; // prints the position of the selected sensor
		// return shoulderFalconSensor.getIntegratedSensorAbsolutePosition();
	}

	/**
	 * @return angle (double) degrees of the second linkage from the horizontal
	 */
	public double getElbowAngle() {
		// encoder reads in [-4096, 0], and absolute position is off by 10 degrees 
		// offset  by shoulder angle so that the angle is relative to the horizotal
		return getShoulderAngle() - ((elbowMotor.getSelectedSensorPosition(1) + 1300) * 360/4096);
		// return elbowFalconSensor.getIntegratedSensorAbsolutePosition();
	}

	public void testMoveShoulder(double analogX) {
		if(shouldMotorStop()){
			System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
			return;
		}

		if (Math.abs(analogX) < Constants.ANALOG_DEAD_ZONE) analogX = 0;
		shoulderMotor.set(ControlMode.PercentOutput, -0.3 * analogX);
	}

	public void testMoveElbow(double analogY) {
		if(shouldMotorStop()){
			System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
			return;
		}

		if (Math.abs(analogY) < Constants.ANALOG_DEAD_ZONE) analogY = 0;
		elbowMotor.set(ControlMode.PercentOutput, 0.3 * analogY);
	}

	/**
	 * @param xPos (double) x position of the end effector - METERS
	 * @param yPos (double) y position of the end effector - METERS
	 * @return angles (double[]) array of angles for each arm length
	 */
	double setEndEffector(double xPos, double yPos) {
		if(shouldMotorStop()){
			System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
			return 0;
		}

		double[] angles = positionInverseKinematics(xPos, yPos);
		
		double alpha = Math.toDegrees(angles[0]);
		double beta = Math.toDegrees(angles[1]);

		setShoulder(alpha);
		setElbow(beta);
		
		return angles[0];
	}

	private void setShoulder(double alpha) {
		shoulderMotor.setVoltage(shoulderPID.calculate(getShoulderAngle(), alpha) + shoulderFeedforward.calculate(alpha, 0, 0));
	}

	private void setElbow(double beta) {
		elbowMotor.setVoltage(elbowPID.calculate(getElbowAngle(), beta) + elbowFeedforward.calculate(beta, 0, 0));
	}

	public void moveArm(double leftAnalogX, double leftAnalogY) {
		if(shouldMotorStop()){
			System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
			return;
		}
		
		double[] combinedSpeeds = speedInverseKinematics(leftAnalogX, leftAnalogY);

		// set the motor speeds as a percent 0-1 (normal)
		shoulderMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
		elbowMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
	}

	//TODO: write method
    /**
	 * This will return the desired x, y pos for the given angle of each length
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
     * @param beta (double) - angle of elbow from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the end effector, in meters
	 */
    public double[] forwardKinematics(double alpha, double beta) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(beta));

		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(beta));

		double[] pos = {x, y + .59};
		return pos;
    }

	/**
	 * This will return the desired x, y pos for the given angle of each length - includes wrist
	 * ASSUMES WRIST IS A REVOLVING JOINT ON THE SAME PLANE AS THE OTHER LENGTHS
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
     * @param beta (double) - angle of elbow from horizontal, in degrees
	 * @param theta (double) - angle of wrist from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the end effector, in meters
	 */
	public double[] forwardKinematics(double alpha, double beta, double theta) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(beta)) +
					Constants.WRIST_LENGTH * Math.cos(Math.toRadians(theta));

		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(beta)) +
					Constants.WRIST_LENGTH * Math.sin(Math.toRadians(theta));

		double[] pos = {x, y};
		return pos;
	}


	//TODO: develop code for a 3R arm in case we have a wrist (though it may depend on how we implement the wrist)

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

		double alpha = Math.toRadians(getShoulderAngle());
		double beta = Math.toRadians(getElbowAngle());

		double v1x = l1 * Math.cos(alpha);
		double v1y = l1 * Math.sin(alpha);

		double v2x = l2 * Math.cos(beta);
		double v2y = l2 * Math.sin(beta);

		double w1x, w2x, w1y, w2y;

		// LATERAL MOVEMENT:
		if (v2y > v1y) {
			w1x = 1;
			w2x = -v1y/v2y * w1x;
		} else {
			w2x = 1;
			w1x = -v2y/v1y * w2x;
		}

		// VERTICAL MOVEMENT:
		if (v2x > v1x) {
			w1y = 1;
			w2y = -v1x/v2x * w1y;
		} else {
			w2y = 1;
			w1y = -v2x/v1x * w2y;
		}

		// Scale by controller input:
		w1x *= xSpeed;
		w2x *= xSpeed;

		w1y *= ySpeed;
		w2y *= ySpeed;

		double[] combinedSpeeds = {w1x + w1y, w2x + w2y};

        return combinedSpeeds;
    }
}
