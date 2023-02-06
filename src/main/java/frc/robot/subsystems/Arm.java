package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;


public class Arm {

    // define objects
	TalonSRX leftTalon, rightTalon;
	VictorSPX leftVictor, rightVictor;
	CANSparkMax leftSparkMax;
	RelativeEncoder sparkMaxEncoder;

    // amount to increment constant during PID Tuning Mode
	double[] PIDIncrements = {0.05, 0.05, 0.05}; // kP, kI, kD increments

	// PID constants
	double[] PIDConstants = {1, 0, 0}; // kP, kI, kD

	// Upper bound for PID constants
	double[] PIDMaximums = {2, 1, 1};
	
	PIDController shoulderPID;
	PIDController elbowPID;

    // current constant to be tuned
	int currentPIDConstant = 0; // 0 = kP, 1 = kI, 2 = kD

	// during PID Tuning Mode, either increasing or decreasing the constants by the increment
	boolean increasingPIDConstant = true;

    /**
	 * Constructor for Arm Class -- setup as tankdrive until I figure out what motors we're using 
	 * 
	 * @param leftMotorPort
	 * @param rightMotorPort
	 */
	public Arm(int leftTalonPort, int leftVictorPort,
                    int rightTalonPort, int rightVictorPort) {

        leftVictor = new VictorSPX(leftVictorPort);
        leftSparkMax = new CANSparkMax(leftTalonPort, CANSparkMaxLowLevel.MotorType.kBrushed);
        rightTalon = new TalonSRX(rightTalonPort);
        rightVictor = new VictorSPX(rightVictorPort);
        sparkMaxEncoder = leftSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, Constants.ENCODER_CPR);

        shoulderPID = new PIDController(PIDConstants[0], PIDConstants[1], PIDConstants[2]);
        elbowPID = new PIDController(PIDConstants[0], PIDConstants[1], PIDConstants[2]);
    }

	public double getAlpha() {
		return 0;
	}

	public double getBeta() {
		return 0;
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
        return 0;
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

		double x1 = l/dist * xPos - h/dist * yPos;
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
