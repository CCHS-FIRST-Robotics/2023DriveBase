package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

	PIDController shoulderVelocityPID, elbowVelocityPID;
	ArmFeedforward shoulderVelocityFeedforward, elbowVelocityFeedforward;

    // current constant to be tuned
	int currentPIDConstant = 0; 

	// during PID Tuning Mode, either increasing or decreasing the constants by the increment
	boolean increasingPIDConstant = true;

	public boolean motorLimits = true;

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


		// Positional PID Controllers
        shoulderPID = new PIDController(Constants.SHOULDER_KP, Constants.SHOULDER_KI, Constants.SHOULDER_KD);
        elbowPID = new PIDController(Constants.ELBOW_KP, Constants.ELBOW_KI, Constants.ELBOW_KD);
		// Velocity PID Controllers
		shoulderVelocityPID = new PIDController(Constants.SHOULDER_VELOCITY_KP, Constants.SHOULDER_VELOCITY_KI, Constants.SHOULDER_VELOCITY_KD);
        elbowVelocityPID = new PIDController(Constants.ELBOW_VELOCITY_KP, Constants.ELBOW_VELOCITY_KI, Constants.ELBOW_VELOCITY_KD);

		// Positional Feedforward Controllers
		shoulderFeedforward = new ArmFeedforward(Constants.SHOULDER_KS, Constants.SHOULDER_KG, Constants.SHOULDER_KV, Constants.SHOULDER_KA);
		elbowFeedforward = new ArmFeedforward(Constants.ELBOW_KS, Constants.ELBOW_KG, Constants.ELBOW_KV, Constants.ELBOW_KA);
		//Velocity Feedforward Controllers
		shoulderVelocityFeedforward = new ArmFeedforward(Constants.SHOULDER_VELOCITY_KS, Constants.SHOULDER_VELOCITY_KG, Constants.SHOULDER_VELOCITY_KV, Constants.SHOULDER_VELOCITY_KA);
		elbowVelocityFeedforward = new ArmFeedforward(Constants.ELBOW_VELOCITY_KS, Constants.ELBOW_VELOCITY_KG, Constants.ELBOW_VELOCITY_KV, Constants.ELBOW_VELOCITY_KA);
    }

	/**
	 * toggles whether motor limits are activated - called when X and Y are pressed simultaneously on the xbox controller
	 */
	public void toggleMotorCheck() {
		motorLimits = !motorLimits;
	}

	/**
	 * @return shouldMotorStop (boolean) returns true if the motor is past any of the limits - ignores the limits if motorLimits is false
	 */
	public boolean shouldMotorStop() {
		double alpha = getShoulderAngle();
		double beta = getElbowAngle();
		double x = forwardKinematics(alpha, beta)[0];
		double y = forwardKinematics(alpha, beta)[1];

		return (
			alpha < Constants.minAlpha |
			alpha > Constants.maxAlpha |

			beta < Constants.minBeta | 
			beta > Constants.maxBeta |

			x < Constants.minX |
			x > Constants.maxX |

			y < Constants.minY |
			y > Constants.maxY |

			(Constants.isInFrameX(x) & Constants.isBelowFrame(y)) |
			(Constants.isInFrameX(x) & Constants.isBelowElectricalBoard(y) & x < 0)
		) & motorLimits;
	}

	/**
	 * Sets the motor output to 0 and sets the motor to brake mode
	 */
	public void stopMotors() {
		shoulderMotor.set(ControlMode.PercentOutput, 0);
		elbowMotor.set(ControlMode.PercentOutput, 0);

		shoulderMotor.setNeutralMode(NeutralMode.Brake);
		elbowMotor.setNeutralMode(NeutralMode.Brake);
	}


	// dont think we actually need this tbh but ill leave it in here in case we decide to use it
	public void setMotorLimits() {
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minAlpha + 2048);
		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.maxAlpha + 2048);
		

		shoulderMotor.configForwardSoftLimitThreshold(4096/360 * Constants.minBeta);
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
		return getShoulderAngle() - ((elbowMotor.getSelectedSensorPosition(1) + 1300) * 360/4096) - 57;

		// return elbowFalconSensor.getIntegratedSensorAbsolutePosition();
	}

	// TODO probably gotta fix units and maybe just calculate it manually tbh - havent tested yet
	/**
	 * @return angle (double) angular velocity of the shoulder joint
	 */
	public double getShoulderAngularVelocity() {
		return shoulderMotor.getSelectedSensorVelocity(1) * 360/4096;
	}

	/**
	 * @return angle (double) angular velocity of the elbow joint
	 */
	public double getElbowAngularVelocity() {
		return elbowMotor.getSelectedSensorVelocity(1) * 360/4096;
	}


	public void testMoveShoulder(double analogX) {
		if (Math.abs(analogX) < Constants.ANALOG_DEAD_ZONE) analogX = 0;

		shoulderMotor.set(ControlMode.PercentOutput, -0.3 * analogX);
	}

	public void testMoveElbow(double analogY) {
		if (Math.abs(analogY) < Constants.ANALOG_DEAD_ZONE) analogY = 0;

		elbowMotor.set(ControlMode.PercentOutput, 0.3 * analogY);
	}

	/**
	 * @param xPos (double) x position of the end effector - METERS
	 * @param yPos (double) y position of the end effector - METERS
	 * @return angles (double[2]) array of angles for each arm length
	 */
	public double setEndEffector(double xPos, double yPos) {
		double[] angles = positionInverseKinematics(xPos, yPos);
		
		double alpha = Math.toDegrees(angles[0]);
		double beta = Math.toDegrees(angles[1]);

		setShoulder(alpha);
		setElbow(beta);
		
		return angles[0];
	}

	/**
	 * Uses the PID and feedforward control loops to set the shoulder at given setpoint
	 * 
	 * @param alpha (double) setpoint for the shoulder angle, in degrees
	 */
	private void setShoulder(double alpha) {
		// System.out.println(
		// 	shoulderFeedforward.calculate(alpha, 0, 0)
		// );
		// System.out.println(
		// 	shoulderPID.calculate(getShoulderAngle(), alpha)
		// );
		shoulderMotor.setVoltage(
			-shoulderPID.calculate(getShoulderAngle(), alpha) + 
			shoulderFeedforward.calculate(alpha, 0, 0)
		);
	}

	/**
	 * Uses the PID and feedforward control loops to set the elbow at given setpoint
	 * 
	 * @param beta (double) setpoint for the elbow angle, in degrees
	 */
	private void setElbow(double beta) {
		elbowMotor.setVoltage(
			-elbowPID.calculate(getElbowAngle(), beta) + 
			elbowFeedforward.calculate(beta, 0, 0)
		);
	}

	// TODO: copilot just spat this out but like maybe they just do the control loops for you??? goateedd
	// TODO: check units match with getShoulderAngularVelocity() and getElbowAngularVelocity()
	private void setShoulderVelocity(double velocity) {
		shoulderMotor.set(ControlMode.Velocity, velocity);
	}
	
	private void setElbowVelocity(double velocity) {
		elbowMotor.set(ControlMode.Velocity, velocity);
	}

	// Uses custom PID/Feedforward control loops - using the useless input so I can use overload and make it easier to switch between the two when testing
	//TODO: check units match with getShoulderAngularVelocity() and getElbowAngularVelocity()
	private void setShoulderVelocity(double velocity, double useless) {
		shoulderMotor.setVoltage(
			shoulderVelocityPID.calculate(getShoulderAngularVelocity(), velocity) +
			shoulderVelocityFeedforward.calculate(0, velocity, 0) // this really seems like it wont work but idk
		);
	}
	
	private void setElbowVelocity(double velocity, double useless) {
		elbowMotor.setVoltage(
			elbowVelocityPID.calculate(getElbowAngularVelocity(), velocity) +
			elbowVelocityFeedforward.calculate(0, velocity, 0) // this really seems like it wont work but idk
		);
	}

	/**
	 * This moves the arm at the given input speed in the horizontal and vertical directions
	 * 
	 * @param leftAnalogX (double) - [0, 1] controller X linear velocity input
	 * @param leftAnalogY (double) - [0, 1] controller Y linear velocity input
	 */
	public void moveArm(double leftAnalogX, double leftAnalogY) {
		double[] combinedSpeeds = speedInverseKinematics(leftAnalogX, leftAnalogY);

		// set the motor speeds as a percent 0-1 (normal) - leaving it commented out so I can test the velocity control
		// shoulderMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
		// elbowMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);

		// TODO: fix units - should be change encoder ticks per 100ms
		setShoulderVelocity(combinedSpeeds[0]);
		setElbowVelocity(combinedSpeeds[1]);
	}

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

		double[] pos = {x, y + .59}; // shoulder joint is ~.59 meters off the ground
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
		double l = (l1*l1 - l2*l2 + dist*dist) / (2*dist);
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
