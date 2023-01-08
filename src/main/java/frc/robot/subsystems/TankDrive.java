package frc.robot.subsystems;

import javax.print.DocFlavor.STRING;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Rotation2d;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;

/**
 * Manages the tank drive base
 * 
 */
public class TankDrive {
	
	// between 0 and 1 - 1 would be full max speed, 0.5 would be half speed, etc
	double maxSpeed = 0.5;

	// scales the rotational speed of the robot, controlled by the right analog
	double rotationalSpeedMultiplier = 0.75;

	// define objects
	TalonSRX leftTalon, rightTalon;
	VictorSPX leftVictor, rightVictor;
	CANSparkMax leftSparkMax;
	RelativeEncoder sparkMaxEncoder;

	DifferentialDriveOdometry odometer;
	AHRS navx;

	// different modes
	String currentMode;
	final String DEFAULT_MODE = "DEFAULT";
	final String DEBUG_MODE = "DEBUG";
	final String PID_TUNING_MODE = "PIDTUNING";
	final String STOP_MODE = "STOP";

	// debug mode variables
    int debugEnabledWheel = 0;

	// PID variables

	// amount to increment constant during PID Tuning Mode
	double[] PIDIncrements = {0.05, 0.05, 0.05}; // kP, kI, kD increments

	// PID constants
	double[] PIDConstants = {1, 0, 0}; // kP, kI, kD

	// Upper bound for PID constants
	double[] PIDMaximums = {2, 1, 1};
	
	PIDController leftPID;
	PIDController rightPID;
	
	// current constant to be tuned
	int currentPIDConstant = 0; // 0 = kP, 1 = kI, 2 = kD

	// during PID Tuning Mode, either increasing or decreasing the constants by the increment
	boolean increasingPIDConstant = true;

	double maxAngularVel = 53; // determined experimentally

	/**
	 * Constructor for TankDrive Class
	 * 
	 * @param leftMotorPort
	 * @param rightMotorPort
	 */
	public TankDrive(int leftTalonPort, int leftVictorPort,
					 int rightTalonPort, int rightVictorPort) {
		leftVictor = new VictorSPX(leftVictorPort);
		leftSparkMax = new CANSparkMax(leftTalonPort, CANSparkMaxLowLevel.MotorType.kBrushed);
		rightTalon = new TalonSRX(rightTalonPort);
		rightVictor = new VictorSPX(rightVictorPort);
		sparkMaxEncoder = leftSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, Constants.ENCODER_CPR);

		currentMode = DEFAULT_MODE;

		leftPID = new PIDController(PIDConstants[0], PIDConstants[1], PIDConstants[2]);
		rightPID = new PIDController(PIDConstants[0], PIDConstants[1], PIDConstants[2]);

		navx = new AHRS(SPI.Port.kMXP);
		odometer = new DifferentialDriveOdometry(new Rotation2d());
		resetPosition();
	}
	
	/**
	 * Drive the robot tank base from controller input 
	 * 
	 * These are doubles on the interval [-1,1], and come from the controller's 
	 * 	analog sticks (circle spinny things)
	 * @param leftAnalogX 
	 * @param leftAnalogY 
	 * @param rightAnalogX 
	 * @param rightAnalogY 
	 */
	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {

		/**
		 * we only care about left Y and right X
		 * left Y is average velocity
		 * right X is velocity difference between wheels
		*/

		// TODO: test multiplying by 2 (to have a larger difference between wheel speeds)
		double x = rightAnalogX; 
		
		double y = leftAnalogY;

		// make sure that both velocities are in [-1, 1]
		double preScaledLeftVel = y - x * rotationalSpeedMultiplier;
		double preScaledRightVel = y + x * rotationalSpeedMultiplier;
		double scaleFactor = 1 / Math.max(Math.max(Math.abs(preScaledLeftVel), Math.abs(preScaledRightVel)), 1);
		
		// scale to what the controller asks
		double leftVel = preScaledLeftVel * scaleFactor * maxSpeed;
		double rightVel = preScaledRightVel * scaleFactor * maxSpeed;
		
		// // leftTalon.set(ControlMode.PercentOutput, leftVel);
		// leftVictor.set(ControlMode.PercentOutput, leftVel);
		// leftSparkMax.set(leftVel);
		// rightTalon.set(ControlMode.PercentOutput, rightVel);
		// rightVictor.set(ControlMode.PercentOutput, rightVel);
		// set the motor speeds
        if(currentMode.equals(DEFAULT_MODE) || currentMode.equals(PID_TUNING_MODE)) {
			leftVictor.set(ControlMode.PercentOutput, -1 *leftVel);
			leftSparkMax.set(leftVel);
			rightTalon.set(ControlMode.PercentOutput, rightVel);
			rightVictor.set(ControlMode.PercentOutput, rightVel);



			// PID stuff

			// actual angular velocities converted to radians per second
			double leftAngVel = sparkMaxEncoder.getVelocity() * Constants.SPARK_MAX_CONVERSION_FACTOR;
			double rightAngVel = rightTalon.getSelectedSensorVelocity() * Constants.TALON_CONVERSION_FACTOR;

			// normalized (actual) angular velocities
			double normalLeftAngVel = leftAngVel / maxAngularVel;
			double normalRightAngVel = rightAngVel / maxAngularVel;


			// set the motors according to the PID
			double leftPIDValue = leftPID.calculate(normalLeftAngVel, leftVel);
			double rightPIDValue = rightPID.calculate(normalRightAngVel, rightVel);


			leftVictor.set(ControlMode.PercentOutput, -1 * leftPIDValue);
			leftSparkMax.set(leftPIDValue);
			
			// System.out.println("Analog: " + leftAnalogY + ", Input: " + rightVel + ", Measured: " + rightAngVel + ", Normalized: " + normalRightAngVel + ", PID: " + rightPIDValue);
			System.out.println("Right: " + normalRightAngVel + ", Left: " + normalLeftAngVel);


			rightTalon.set(ControlMode.PercentOutput, rightPIDValue);
			rightVictor.set(ControlMode.PercentOutput, rightPIDValue);

			

        } else if (currentMode.equals(DEBUG_MODE)) {
            switch (debugEnabledWheel){
                case 0:
                    leftVictor.set(ControlMode.PercentOutput, -1 * leftVel);
                    break;
                case 1:
                    rightVictor.set(ControlMode.PercentOutput, rightVel);
                    break;
                case 2:
                    rightTalon.set(ControlMode.PercentOutput, rightVel);
                    break;
                case 3:
                    leftSparkMax.set(leftVel);
                    break;
            }
        } else if (currentMode.equals(STOP_MODE)){
			// STOP!!!!! set motors to 0
			leftVictor.set(ControlMode.PercentOutput, 0);
			leftSparkMax.set(0);
			rightTalon.set(ControlMode.PercentOutput, 0);
			rightVictor.set(ControlMode.PercentOutput, 0);
		}
		updatePosition();
		// System.out.println(rightTalon.getSelectedSensorVelocity()); // clicks per 100ms
		// System.out.println(rightTalon.getSelectedSensorVelocity() * 10 * 60 / 4096);
		// System.out.println(sparkMaxEncoder.getVelocity()); // actual rpm
		// System.out.println("");




		// // Print angular velocity and motor use level
		// // get rotational speeds from the motors on each side
		// // divide by 60 to get rotations per second
		// double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;

		// if (Math.abs(leftRPS) > 0.1){
		// 	// // multiply by 10 because this is per 100ms - we want rps
		// 	double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;
		// 	System.out.println("Right Rps: " + rightRPS + "\nLeft RPS: " + leftRPS);
		// 	System.out.println("Right velocity from controller: " + rightVel + "\nLeft Velocity from controller: " + leftVel);
		// }
		 

	}

	/**
	 * The speed bracket controls the multiplier for all the speeds 
	 * So when you change it, lets say, to 1/2 speed, all directions will be
	 * 	at 1/2 speed
	 */
	public void increaseSpeedBracket() {
		maxSpeed = Math.min(0.8, maxSpeed + 0.1);
	}
  
	public void decreaseSpeedBracket() {
		maxSpeed = Math.max(0.2, maxSpeed - 0.1);
	}

	public void turnOnDefaultMode() {
		if(currentMode.equals(DEFAULT_MODE)) return;
		currentMode = DEFAULT_MODE;
		System.out.println("DEFAULT MODE");
	}

	public void turnOnDebugMode() {
		if(currentMode.equals(DEBUG_MODE)) return;
        currentMode = DEBUG_MODE;
        System.out.println("DEBUG MODE");
    }

	public void turnOnStopMode() {
		if(currentMode.equals(STOP_MODE)) return;
		currentMode = STOP_MODE;
		System.out.println("STOP MODE");
	}

	public void turnONPIDTurningMode() {
		if(currentMode.equals(PID_TUNING_MODE)) return;
		currentMode = PID_TUNING_MODE;
		System.out.println("PID TUNING MODE");
	}

    public void cycleWheelDebugMode() {
        debugEnabledWheel++;
        debugEnabledWheel %= 4;
        System.out.println("Current Wheel: " + debugEnabledWheel);
    }

	public void updateRobotVelocity() {
		
		// get rotational speeds from the motors on each side
		// divide by 60 to get rotations per second
		// double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;
		// // multiply by 10 because this is per 100ms - we want rps
		// double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;

		// // find linear velocities in m/s
		// double leftVel = (leftRPS * 2 * Math.PI) * Constants.TANK_WHEEL_RADIUS;
		// double rightVel = (rightRPS * 2 * Math.PI) * Constants.TANK_WHEEL_RADIUS;

		// currentSpeeds.leftMetersPerSecond = leftVel;
		// currentSpeeds.rightMetersPerSecond = rightVel;
	}

	public void getPosition()
	{

	}

	public void updatePosition()
	{
		// get the linear position of the left wheel and convert to meters
		double leftWheelPos = -1 * sparkMaxEncoder.getPosition() * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the linear position of the right wheel and convert to meters
		double rightWheelPos = rightTalon.getSelectedSensorPosition() / Constants.ENCODER_CPR * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the angle of the robot
		double theta = navx.getAngle() / 180 * Math.PI;
		Rotation2d rot = new Rotation2d(theta);
		odometer.update(rot, leftWheelPos, rightWheelPos);
	}

	public void AButtonPressed() {
		switch(currentMode) {
			case DEFAULT_MODE:
				printPosition();
				break;
			case DEBUG_MODE:
				printPosition();	
				break;
			case STOP_MODE:
				printPosition();	
				break;
			case PID_TUNING_MODE:
				incrementPIDConstant();
				break;
		}
	}

	public void BButtonPressed() {
		switch(currentMode) {
			case DEFAULT_MODE:
				resetPosition();
				break;
			case DEBUG_MODE:
				resetPosition();	
				break;
			case STOP_MODE:
				resetPosition();
				break;
			case PID_TUNING_MODE:
				cyclePIDConstant();
				break;
		}
	}

	public void XButtonPressed() {
		switch(currentMode) {
			case PID_TUNING_MODE:
				toggleDecreasingPIDIncrement();
				break;
		}
	}

	public void YButtonPressed() {
		switch(currentMode) {
			case PID_TUNING_MODE:
				printPIDConstants();
				break;
		}
	}

	/**
	 * PID Tuning Mode: Increments the selected PID constant
	 */
	public void incrementPIDConstant() {
		if(increasingPIDConstant) {
			PIDConstants[currentPIDConstant] += PIDIncrements[currentPIDConstant];
		} else {
			PIDConstants[currentPIDConstant] -= PIDIncrements[currentPIDConstant];
		}
		// make sure constants are in [0, constantMax]
		PIDConstants[currentPIDConstant] = Math.min(PIDConstants[currentPIDConstant], PIDMaximums[currentPIDConstant]);
		PIDConstants[currentPIDConstant] = Math.max(PIDConstants[currentPIDConstant], 0);

		// print PID constants
		System.out.println("kP: " + PIDConstants[0] + ", kI: " + PIDConstants[1] + ", kD: " + PIDConstants[2]);

		// set PID constant in the PID controllers
		if(currentPIDConstant == 0) {
			leftPID.setP(PIDConstants[0]);
			rightPID.setP(PIDConstants[0]);
		} else if (currentPIDConstant == 1) {
			leftPID.setI(PIDConstants[1]);
			rightPID.setI(PIDConstants[1]);
		} else if (currentPIDConstant == 2) {
			leftPID.setD(PIDConstants[2]);
			rightPID.setD(PIDConstants[2]);
		}
	}

	/**
	 * PID Tuning Mode: Cycles between the PID constants
	 */
	public void cyclePIDConstant() {
		currentPIDConstant++;
		currentPIDConstant %= 3;
		if(currentPIDConstant == 0) {
			System.out.println("kP Selected");
		}
		else if(currentPIDConstant == 1) {
			System.out.println("kI Selected");
		}
		else if(currentPIDConstant == 2) {
			System.out.println("kD Selected");
		}
	}

	/*
	 * PID Tuning Mode: Toggles between increasing and decreasing the PID constants on each increment
	 */
	public void toggleDecreasingPIDIncrement() {
		if(increasingPIDConstant) {
			increasingPIDConstant = false;
			System.out.println("Decreasing PID Constants");
		} else {
			increasingPIDConstant = true;
			System.out.println("Increasing PID Constants");
		}
	}

	public void printPIDConstants() {
		System.out.println("kP: " + PIDConstants[0] + ", kI: " + PIDConstants[1] + ", kD: " + PIDConstants[2]);
	}

	public void resetPosition()
	{
		// reset the sensors to zero the position
		navx.reset();
		navx.resetDisplacement();
		sparkMaxEncoder.setPosition(0);
		rightTalon.setSelectedSensorPosition(0);
		odometer.resetPosition(new Pose2d(), new Rotation2d());
	}

	public void printPosition()
	{
		Pose2d posFromWheelDisplacement = odometer.getPoseMeters();
		System.out.println("Position from odometer and wheel: ");
		System.out.println("X: " + posFromWheelDisplacement.getX() + " Y: " + posFromWheelDisplacement.getY() + " rotation: " + posFromWheelDisplacement.getRotation());

		System.out.println("\nPosition info from navx: ");
		System.out.println("X: " + navx.getDisplacementX() + " Y: " + navx.getDisplacementY() + " rotation: " + navx.getAngle());
		System.out.println();

		// get the linear position of the left wheel and convert to meters
		double leftWheelPos = -1 * sparkMaxEncoder.getPosition() * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the linear position of the right wheel and convert to meters
		double rightWheelPos = rightTalon.getSelectedSensorPosition() / Constants.ENCODER_CPR * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;
		System.out.println("Right Pos: " + rightWheelPos + " left pos: " + leftWheelPos);


		// Print angular velocity and motor use level
		// get rotational speeds from the motors on each side
		// divide by 60 to get rotations per second
		double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;
		// // multiply by 10 because this is per 100ms - we want rps
		double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;
		System.out.println("Right Rps: " + rightRPS + "\nLeft RPS: " + leftRPS);

	}
	
}
