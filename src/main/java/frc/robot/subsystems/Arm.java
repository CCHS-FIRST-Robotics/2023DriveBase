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


    //TODO: write method
    /**
	 * This will return the desired angle of each arm length using IK
	 * 
	 * @param xPos (double)
     * @param yPos (double)
	 * @return angles
	 */
    private double inverseKinematics(double xPos, double yPos) {
        return 0;
    }
}
