package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.lang.Math;


public class MecaSubsystem extends SubsystemBase {

	// TODO: create better modules for teleop and autonomous driving / odometry

	// used to scale speeds - 1 would be max speed, 0.5 would be half speed, etc.
	double speedMultiplier = 0.6;

	// different modes
	enum Mode {
		DEFAULT_MODE,
		DEBUG_MODE,
		PID_TUNING_MODE,
		STOP_MODE
	}
	Mode currentMode = Mode.DEFAULT_MODE;

	// the motor to be activated during debug mode
	int debugEnabledMotor = 0;

	// Odometry class for tracking robot pose
	final MecanumDriveOdometry mOdom;

	// Drive Object
	MecanumDrive mDrive;

	// Motor Controller Objects
	WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

	// Motor Sensor Objects
	TalonFXSensorCollection flFalconSensor, rlFalconSensor, frFalconSensor, rrFalconSensor;

	// Motor positions Object
	MecanumDriveWheelPositions wheelPositions;

	MecaSubsystem(int frontLeftMotorPort, int frontRightMotorPort,
					int rearLeftMotorPort, int rearRightMotorPort) {

		frontLeftMotor = new WPI_TalonFX(frontLeftMotorPort);
		frontRightMotor = new WPI_TalonFX(frontRightMotorPort);
		rearLeftMotor = new WPI_TalonFX(rearLeftMotorPort);
		rearRightMotor = new WPI_TalonFX(rearRightMotorPort);

		// invert motors to make forward the right direction
		frontRightMotor.setInverted(true);
		rearRightMotor.setInverted(true);


		mDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		
		// Initializing Falcon sensors
		flFalconSensor = new TalonFXSensorCollection(frontLeftMotor);
		rlFalconSensor = new TalonFXSensorCollection(rearLeftMotor);
		frFalconSensor = new TalonFXSensorCollection(frontRightMotor);
		rrFalconSensor = new TalonFXSensorCollection(rearRightMotor);

		// Odometry: !!secondary constructor takes initialPose argument
		mOdom = new MecanumDriveOdometry(Constants.MECANUM_KINEMATICS, new Rotation2d(Math.toRadians(SmartDashboard.getNumber("NavHead", 0))), getWheelPositions());
	}
	
	void drive(double speedX, double speedY, double rotateSpeed) {

		switch (currentMode){
			case STOP_MODE:
				// STOP!!!!! set motors to 0
				// slower stop
				speedX = slowDown(speedX);
				speedY = slowDown(speedY);
				rotateSpeed = slowDown(rotateSpeed);
				break;

			// TODO: figure out how to implement a debug mode
			// case DEBUG_MODE:
			// 	// Debug mode (toggle wheels with left stick button)
				
			// 	switch (debugEnabledMotor){
			// 		case 0:
			// 			frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
			// 			break;
			// 		case 1:
			// 			frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
			// 			break;
			// 		case 2:
			// 			rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2]);
			// 			break;
			// 		case 3:
			// 			rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
			// 			break;	
			// 	}

			// 	break;
			case PID_TUNING_MODE:
				// nothing yet
				break;
		}
		
		// method defines Y as left/right and X as forward/backward - contrary to docs, right and forward are positive
		mDrive.driveCartesian(speedY, speedX, rotateSpeed);
	}

	/**
	 * This will return a value lower than the input, and it is used to slow 
	 * down the motors during stop mode
	 * 
	 * @param inputVelocity between -1 and 1 (double)
	 * @return the new value (lower)
	 */
	private double slowDown(double inputVelocity){
		// velocity needs to be reduced
		double newVelocity = inputVelocity / Constants.SLOW_DOWN_FACTOR;

		// input has reached cutoff, now returning 0 speed
		if (Math.abs(inputVelocity) < Constants.SLOW_DOWN_CUTOFF){
			newVelocity = 0;
		}
		
		return newVelocity;
	}

	public void printControlsOfCurrentMode() {
		System.out.println("Controls:");
		switch(currentMode) {
			case DEFAULT_MODE:
				System.out.println("Left Bracket: Decrease speed multiplier");
				System.out.println("Right Bracket: Increase speed multiplier");
				break;
			case DEBUG_MODE:
				System.out.println("A: Cycle active motor");
				System.out.println("B: Print current active motor");
				System.out.println("Left Bracket: Decrease speed multiplier");
				System.out.println("Right Bracket: Increase speed multiplier");
				break;
			case STOP_MODE:
				System.out.println("Left Bracket: Decrease speed multiplier");
				System.out.println("Right Bracket: Increase speed multiplier");
				break;
			case PID_TUNING_MODE:
				System.out.println("Left Bracket: Decrease speed multiplier");
				System.out.println("Right Bracket: Increase speed multiplier");
				break;
		}
	}

	/**
	 * The speed bracket controls the multiplier for al the speeds
	 * So when you change it, lets say, to 1/2 speed, all movement will be at
	 * 1/2 speed
	 */
	public void increaseSpeedBracket() {
		speedMultiplier = Math.min(1, speedMultiplier + 0.1);
		mDrive.setMaxOutput(speedMultiplier);
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void decreaseSpeedBracket() {
		// the min is 0.2 because below that the robot is unlikely to move
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
		mDrive.setMaxOutput(speedMultiplier);
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void turnOnDefaultMode() {
		if(currentMode == Mode.DEFAULT_MODE) return;
		currentMode = Mode.DEFAULT_MODE;
		System.out.println("********************************");
		System.out.println("Current Mode: DEFAULT Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
	}

	public void turnOnDebugMode() {
		if(currentMode == Mode.DEBUG_MODE) return;
        currentMode = Mode.DEBUG_MODE;
		System.out.println("********************************");
        System.out.println("Current Mode: DEBUG Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
    }

	/**
	 * Cycle between each motor during debug mode
	 */
	public void cycleMotor() {
		if (currentMode == Mode.DEBUG_MODE) {
			debugEnabledMotor++;
        	debugEnabledMotor %= 4;
        	System.out.println("Current Motor: " + debugEnabledMotor);
		}
	}

	/**
	* prints the number of the currently activated motors during debug mode
	*/
	public void printActiveMotorDebugMode() {
		if (currentMode == Mode.DEBUG_MODE) {
			System.out.println("Current Motor: " + debugEnabledMotor);
		}
	}

	
	public void turnOnStopMode() {
		if(currentMode == Mode.STOP_MODE) return;
		currentMode = Mode.STOP_MODE;

		System.out.println("STOP MODE");
	}
		
	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		mOdom.update(new Rotation2d(Math.toRadians(SmartDashboard.getNumber("NavHead", 0))), getWheelPositions());
	}

	/**
	 * Returns the total distances measured by each motor
	 * 
	 * @return wheel positions
	 */
	MecanumDriveWheelPositions getWheelPositions() {

		// TODO: determine whether should use absolute position or just position

		return new MecanumDriveWheelPositions(flFalconSensor.getIntegratedSensorAbsolutePosition(), 
												frFalconSensor.getIntegratedSensorAbsolutePosition(), 
												rlFalconSensor.getIntegratedSensorAbsolutePosition(),
												rrFalconSensor.getIntegratedSensorAbsolutePosition());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	Pose2d getPose() {
		return mOdom.getPoseMeters();
	}

	// TODO: create method that returns wheel speeds of the robot
	// TODO: create mthod that allows control of wheels with voltages
	// TODO: method of averages of encoder distances

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	void resetOdometry(Pose2d pose) {
		//resetEncoders();
		mOdom.resetPosition(
			new Rotation2d(Math.toRadians(SmartDashboard.getNumber("NavHead", 0))), getWheelPositions(), pose);
	}

	
	
	// TODO: create structure for odometry

}

