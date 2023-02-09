package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import java.lang.Math;


public class MecaDrive extends DriveBase {

	// TODO: create better modules for teleop and autonomous driving / odometry

	IMU imu;

	// the motor to be activated during debug mode
	int debugEnabledMotor = 0;

	// Odometry class for tracking robot pose
	final MecanumDriveOdometry mOdom;

	// Drive Object
	MecanumDrive mDrive;

	// Motor Controller Objects
	WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

	// Motor positions Object
	MecanumDriveWheelPositions wheelPositions;

	public MecaDrive(int frontLeftMotorPort, int frontRightMotorPort,
					int rearLeftMotorPort, int rearRightMotorPort, IMU imu) {

		frontLeftMotor = new WPI_TalonFX(frontLeftMotorPort);
		frontRightMotor = new WPI_TalonFX(frontRightMotorPort);
		rearLeftMotor = new WPI_TalonFX(rearLeftMotorPort);
		rearRightMotor = new WPI_TalonFX(rearRightMotorPort);

		// invert motors to make forward the right direction
		frontRightMotor.setInverted(true);
		rearRightMotor.setInverted(true);


		mDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

		this.imu = imu;

		// Odometry: !!secondary constructor takes initialPose argument
		mOdom = new MecanumDriveOdometry(Constants.MECANUM_KINEMATICS, new Rotation2d(Math.toRadians(imu.getAngle())), getWheelPositions());
	}
	
	@Override
	public void drive(double speedX, double speedY, double rotateSpeed) {
		speedX *= speedMultiplier;
		speedY *= speedMultiplier;
		rotateSpeed *= speedMultiplier;

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
	 * Cycle between each motor during debug mode
	 */
	@Override
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
		
	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		mOdom.update(new Rotation2d(Math.toRadians(imu.getAngle())), getWheelPositions());
	}

	/**
	 * Converts raw position units to meters
	 * @param rawPosition the position from an encoder in raw sensor units
	 * @return the position in meters
	 */
	private double convertPosition(double rawPosition) {
		// raw units are "clicks," so divide by "clicks" per rotation to get rotations
		double position = rawPosition / Constants.ENCODER_CPR;
		// multiply by circumference to get linear distance
		position *= Math.PI * Constants.MECANUM_WHEEL_DIAMETER;
		return position;
	}

	/**
	 * Returns the total distances measured by each motor
	 * 
	 * @return wheel positions
	 */
	MecanumDriveWheelPositions getWheelPositions() {

		// TODO: determine whether should use absolute position or just position

		return new MecanumDriveWheelPositions(convertPosition(frontLeftMotor.getSelectedSensorPosition()), 
											  convertPosition(frontRightMotor.getSelectedSensorPosition()),
											  convertPosition(rearLeftMotor.getSelectedSensorPosition()),
											  convertPosition(rearRightMotor.getSelectedSensorPosition()));
	}

	/**
	 * Converts raw sensor velocity to meters/second
	 * @param rawVelocity the velocity from an encoder in raw sensor units
	 * @return velocity in m/s
	 */
	private double convertVelocity(double rawVelocity) {
		// convert to rotations per second because raw units are "clicks" per 100ms
		double velocity = rawVelocity / Constants.ENCODER_CPR * 10;
		// multiply by circumference to get linear velocity
		velocity *= Math.PI * Constants.MECANUM_WHEEL_DIAMETER;
		return velocity;
	}

	/**
	 * Returns the current wheel speeds of each motor
	 * 
	 * @return wheel speeds
	 */
	MecanumDriveWheelSpeeds getWheelSpeeds() {
		return new MecanumDriveWheelSpeeds(convertVelocity(frontLeftMotor.getSelectedSensorVelocity()),
										   convertVelocity(frontRightMotor.getSelectedSensorVelocity()), 
										   convertVelocity(rearLeftMotor.getSelectedSensorVelocity()),
										   convertVelocity(rearRightMotor.getSelectedSensorVelocity()));
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return mOdom.getPoseMeters();
	}

	public double getOdomX() {
		return mOdom.getPoseMeters().getTranslation().getX();
	}

	public double getOdomY() {
		return mOdom.getPoseMeters().getTranslation().getY();
	}

	public double getOdomHeading() {
		return mOdom.getPoseMeters().getRotation().getDegrees();
	}

	// TODO: create method that returns wheel speeds of the robot
	// TODO: create mthod that allows control of wheels with voltages
	// TODO: method of averages of encoder distances

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		//resetEncoders();
		mOdom.resetPosition(
			new Rotation2d(Math.toRadians(imu.getAngle())), getWheelPositions(), pose);
	}

	// TODO: create structure for odometry

}
