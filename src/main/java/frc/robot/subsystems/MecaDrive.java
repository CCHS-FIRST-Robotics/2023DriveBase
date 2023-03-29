package frc.robot.subsystems;
import frc.robot.*;
import frc.robot.utils.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.lang.Math;

public class MecaDrive extends DriveBase {

	// TODO: create better modules for teleop and autonomous driving / odometry

	IMU imu;
	Arm arm;

	// the motor to be activated during debug mode
	int debugEnabledMotor = 0;

	// Odometry class for tracking robot pose
	final MecanumDriveOdometry mOdom;

	// Drive Object
	public MecanumDrive mDrive;

	// Motor Controller Objects
	public WPI_TalonFX frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

	// Motor positions Object
	MecanumDriveWheelPositions wheelPositions;

	// UKF pose estimator w/ vslam data
	MecanumDrivePoseEstimator poseEstimator;
	Matrix<N3, N1> covarZed = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1, 1, 1);
	Matrix<N3, N1> covarOdom = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(3, 3, 3);
	
	// timer for autonomous
	public Timer autonTimer;

	PIDController rampPID = new PIDController(Constants.RAMP_P, Constants.RAMP_I, Constants.RAMP_D);

	boolean brakeMode = false;

	public double headingSetPoint = 0; // in degrees

	// consider trying a profiled PID controller because that is what the Holonomic Controller expects
	PIDController rotationPID = new PIDController(Constants.ROTATION_PID[0], 
												  Constants.ROTATION_PID[1], 
												  Constants.ROTATION_PID[2]);
	
	public MecaDrive(int frontLeftMotorPort, int frontRightMotorPort,
					int rearLeftMotorPort, int rearRightMotorPort, IMU imu, Arm arm) {

		frontLeftMotor = new WPI_TalonFX(frontLeftMotorPort);
		frontRightMotor = new WPI_TalonFX(frontRightMotorPort);
		rearLeftMotor = new WPI_TalonFX(rearLeftMotorPort);
		rearRightMotor = new WPI_TalonFX(rearRightMotorPort);

		// invert motors to make forward the right direction
		frontRightMotor.setInverted(true);
		rearRightMotor.setInverted(true);

		// zero encoders
		frontLeftMotor.setSelectedSensorPosition(0);
		frontRightMotor.setSelectedSensorPosition(0);
		rearLeftMotor.setSelectedSensorPosition(0);
		rearRightMotor.setSelectedSensorPosition(0);

		configTalonFX(frontLeftMotor);
		configTalonFX(frontRightMotor);
		configTalonFX(rearLeftMotor);
		configTalonFX(rearRightMotor);

		mDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

		this.imu = imu;
		this.arm = arm;

		Rotation2d initialHeading = new Rotation2d(Math.toRadians(imu.getHeading()));
		// Odometry: !!secondary constructor takes initialPose argument
		mOdom = new MecanumDriveOdometry(Constants.MECANUM_KINEMATICS, initialHeading, getWheelPositions());

		// Pose estimator - TODO: specify covariance matrices in secondary constructor
		poseEstimator = new MecanumDrivePoseEstimator(
			Constants.MECANUM_KINEMATICS, 
			new Rotation2d(Math.toRadians(imu.getHeading())), 
			getWheelPositions(),
			new Pose2d(0, 0, initialHeading),
			covarOdom,
			covarZed
		);

		rotationPID.setTolerance(0.5);
	
		// see declaration in DriveBase
		// set up the config for autonomous trajectories
		// see another team's implementation https://github.com/FRC5254/FRC-5254-2020/blob/master/src/main/java/frc/robot/commands/auto/Path.java
		// relevant info! https://www.chiefdelphi.com/t/poll-why-didnt-you-use-the-wpilib-trajectory-generator-this-year/384611/37
		trajectoryConfig = new TrajectoryConfig(Constants.maxVelocityMetersPerSecond, Constants.maxAccelerationMetersPerSecond);
		trajectoryConfig.setKinematics(Constants.MECANUM_KINEMATICS);
		trajectoryConfig.addConstraint( // this may be unnessesary, hmm unsure
			new MecanumDriveKinematicsConstraint(Constants.MECANUM_KINEMATICS, Constants.maxVelocityMetersPerSecond));
	}

	public void drive(double speedX, double speedY, double rotateSpeed, boolean fieldOriented) {
		speedX *= speedMultiplier;
		speedY *= speedMultiplier;
		rotateSpeed *= speedMultiplier;
		// these speeds are linearized from 1 to -1

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
			default:
				break;
		}
		
		// method defines Y as left/right and X as forward/backward - contrary to docs, right and forward are positive
		// https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-drive-subsystem.html
		if (fieldOriented) {
			mDrive.driveCartesian(speedY, speedX, rotateSpeed, new Rotation2d(Math.toRadians(imu.getHeading())));
		} else {
			mDrive.driveCartesian(speedY, speedX, rotateSpeed);
		}
		
	}

	/**
	 * instead of directly using the right joystick input as rotational velocity, use it to adjust the heading setpoint
	 * this means that when the robot is not told to rotate, the heading setpoint will stay the same and the robot should
	 * keep driving straight
	 */
	public void driveStraight(double speedX, double speedY, double rotInput, boolean fieldOriented) {
		speedX *= speedMultiplier;
		speedY *= speedMultiplier;
		rotInput *= speedMultiplier;

		// divide by 5 to make the speed more reasonable (TUNE THIS)
		headingSetPoint += rotInput / 0.087;
		double currentHeading = imu.getAngle();

		// get PID output and clamp to a range of [-1.0, 1.0]
		double rotVel = MathUtil.clamp(rotationPID.calculate(currentHeading, headingSetPoint), -1.0, 1.0);
		double armExtension = arm.getState().getX();
		// LIMIT CENTRIPETAL ACCLERATION - ac = w^2 * r (max at w = 1, r = .35 - inside frame at max speed)
		//								   				imples ac = w^2r <= .35, w <= sqrt(.35/r)
		double maxW = Math.sqrt(.35 / armExtension);
		rotVel = MathUtil.clamp(rotVel, -maxW, maxW);

		// increase pid output so that it rotates for small error (only if we aren't translating significantly)
		if (Math.abs(speedX) < Math.abs(rotVel) && Math.abs(speedY) < Math.abs(rotVel))
			rotVel += Constants.ROTATION_ADJUSTMENT * Math.signum(rotVel);
		// System.out.println("Setpoint: " + headingSetPoint + ", current: " + currentHeading + " PID output: " + rotVel);

		if (fieldOriented) {
			mDrive.driveCartesian(speedY, speedX, rotVel, new Rotation2d(Math.toRadians(currentHeading + 180)));
		} else {
			mDrive.driveCartesian(speedY, speedX, rotVel);
		}
	}

	/**
	 * Drive with ChassisSpeeds object (useful for autonomous)
	 * 
	 * TODO: Tune velocity control? https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html
	 */
	@Override
	public void drive(ChassisSpeeds chassisSpeeds) {
		// System.out.println(chassisSpeeds.vxMetersPerSecond + ", " + chassisSpeeds.vyMetersPerSecond);
		// convert to mecanum speeds (in m/s)
		MecanumDriveWheelSpeeds speeds = Constants.MECANUM_KINEMATICS.toWheelSpeeds(chassisSpeeds);
		// System.out.println(speeds.frontLeftMetersPerSecond + ", "
		// 				   + speeds.frontRightMetersPerSecond + ", "
		// 				   + speeds.rearLeftMetersPerSecond + ", "
		// 				   + speeds.rearRightMetersPerSecond);

		// convert to motor native units (clicks/100ms)
		double conversionFactor = (Constants.FALCON_GEARBOX_RATIO * Constants.TALON_FX_CPR) / (10 * Math.PI * Constants.MECANUM_WHEEL_DIAMETER);
		double FLSpeed = speeds.frontLeftMetersPerSecond * conversionFactor;
		double FRSpeed = speeds.frontRightMetersPerSecond * conversionFactor;
		double RLSpeed = speeds.rearLeftMetersPerSecond * conversionFactor;
		double RRSpeed = speeds.rearRightMetersPerSecond * conversionFactor;

		// System.out.println(FLSpeed + ", "
		// 				   + FRSpeed + ", "
		// 				   + RLSpeed + ", "
		// 				   + RRSpeed);

		// set the motor speeds
		frontLeftMotor.set(ControlMode.Velocity, FLSpeed);
		frontRightMotor.set(ControlMode.Velocity, FRSpeed);
		rearLeftMotor.set(ControlMode.Velocity, RLSpeed);
		rearRightMotor.set(ControlMode.Velocity, RRSpeed);
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

	public void rampAutoBalance() {
		// frontRightMotor.setNeutralMode(NeutralMode.Brake);
		// frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		// rearRightMotor.setNeutralMode(NeutralMode.Brake);
		// rearLeftMotor.setNeutralMode(NeutralMode.Brake);

		// System.out.println(getRampFeedforward());
		// System.out.println(rampPID.calculate(imu.getPitch(), 0));
		double tilt = imu.getTilt();
		if (Math.abs(tilt) < 3) {
			driveStraight(0, 0, 0, true);
		} else {
			driveStraight(0, 
				-rampPID.calculate(tilt, 0) + 
				getRampFeedforward(),
				0, true);
		}
	}

	public void rampHold() {
		// System.out.println("G: " + getRampFeedforward());
		drive(0, getRampFeedforward(), 0, false);
	}

	public void setMotorsNeutralMode(NeutralMode mode) {
		// brakeMode = !brakeMode;
		// if (brakeMode) {
		// 	frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		// 	frontRightMotor.setNeutralMode(NeutralMode.Brake);
		// 	rearLeftMotor.setNeutralMode(NeutralMode.Brake);
		// 	rearRightMotor.setNeutralMode(NeutralMode.Brake);
		// }
		// else {
		// 	frontLeftMotor.setNeutralMode(NeutralMode.Coast);
		// 	frontRightMotor.setNeutralMode(NeutralMode.Coast);
		// 	rearLeftMotor.setNeutralMode(NeutralMode.Coast);
		// 	rearRightMotor.setNeutralMode(NeutralMode.Coast);
		// }
		frontLeftMotor.setNeutralMode(mode);
		frontRightMotor.setNeutralMode(mode);
		rearLeftMotor.setNeutralMode(mode);
		rearRightMotor.setNeutralMode(mode);
	}

	/**
	 * Calculates the voltage required to keep the robot from sliding on the ramp
	 * 
	 * @return controlInput (double) voltage to send to the motors
	 */
	public double getRampFeedforward() {
		return Constants.RAMP_G * Math.sin(Math.toRadians(imu.getPitch()));
	}

	public void updatePose(double[] zedTagPose, double[] zedPoseEstimate, long counter) {
		Rotation2d currentHeading = new Rotation2d(Math.toRadians(imu.getHeading()));
		poseEstimator.updateWithTime(counter/Constants.PERIOD, currentHeading, getWheelPositions());

		if (zedPoseEstimate[0] != -1) {
			Pose2d zedPose = new Pose2d(zedPoseEstimate[0], zedPoseEstimate[1], new Rotation2d(Math.toRadians(zedPoseEstimate[2])));
			poseEstimator.addVisionMeasurement(zedPose, counter/Constants.PERIOD, covarZed);
		}

		if (zedTagPose[0] != -1) {
			// yaw estimate is currently not working so using a covar of inf
			Pose2d tagPose = new Pose2d(zedTagPose[0], zedTagPose[1], new Rotation2d(Math.toRadians(zedTagPose[2])));
			double covar = .01 * zedTagPose[0]*zedTagPose[0] + zedTagPose[1]*zedTagPose[1];
			Matrix<N3, N1> covarTag = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(covar, covar, Integer.MAX_VALUE);
			poseEstimator.addVisionMeasurement(tagPose, counter/Constants.PERIOD, covarTag);
		}
	}

	public void configTalonFX(WPI_TalonFX talon) {
		talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
										   Constants.FALCON_PID_IDX, 
										   Constants.FALCON_TIMEOUT_MS);
		
		talon.configNominalOutputForward(0, Constants.FALCON_TIMEOUT_MS);
		talon.configNominalOutputReverse(0, Constants.FALCON_TIMEOUT_MS);
		talon.configPeakOutputForward(1, Constants.FALCON_TIMEOUT_MS);
		talon.configPeakOutputReverse(-1, Constants.FALCON_TIMEOUT_MS);

		talon.config_kF(Constants.FALCON_PID_IDX, Constants.FALCON_KF, Constants.FALCON_TIMEOUT_MS);
		talon.config_kP(Constants.FALCON_PID_IDX, Constants.FALCON_KP);
		talon.config_kD(Constants.FALCON_PID_IDX, Constants.FALCON_KD);

		talon.config_kI(Constants.FALCON_PID_IDX, Constants.FALCON_KI);
		talon.config_IntegralZone(Constants.FALCON_PID_IDX, Constants.FALCON_INTEGRAL_ZONE);

		talon.configClosedLoopPeakOutput(Constants.FALCON_PID_IDX, 0.5);
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
	

	//TODO: remove since it's never called
	@Override
	public void periodic() {
		// Update the odometry in the periodic block		
	}

	public void updateOdometry() {
		mOdom.update(new Rotation2d(Math.toRadians(imu.getHeading())), getWheelPositions());
	}

	/**
	 * Converts raw position units to meters
	 * @param rawPosition the position from an encoder in raw sensor units
	 * @return the position in meters
	 */
	private double convertPosition(double rawPosition) {
		// raw units are "clicks," so divide by "clicks" per rotation to get rotations
		// also account for gear ratio because the encoders measure motor output, not actual wheel
		double position = rawPosition / (Constants.TALON_FX_CPR * Constants.FALCON_GEARBOX_RATIO);
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
		// also account for gear ratio because the encoders measure motor output, not actual wheel
		double velocity = rawVelocity / (Constants.TALON_FX_CPR * Constants.FALCON_GEARBOX_RATIO) * 10;
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
		return poseEstimator.getEstimatedPosition();
	}

	public double getPoseX() {
		return poseEstimator.getEstimatedPosition().getTranslation().getX();
	}

	public double getPoseY() {
		return poseEstimator.getEstimatedPosition().getTranslation().getY();
	}

	public double getPoseHeading() {
		return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
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

	public void printVelocity() {
		System.out.println(frontLeftMotor.getSelectedSensorVelocity() + ", " + 
						   frontRightMotor.getSelectedSensorVelocity() + ", " + 
						   rearLeftMotor.getSelectedSensorVelocity() + ", " +
						   rearRightMotor.getSelectedSensorVelocity());
	}

	// TODO: create structure for odometry


	public void clearOdom() {
		frontLeftMotor.setSelectedSensorPosition(0);
		frontRightMotor.setSelectedSensorPosition(0);
		rearLeftMotor.setSelectedSensorPosition(0);
		rearRightMotor.setSelectedSensorPosition(0);
		imu.reset();
		imu.resetDisplacement();
		mOdom.resetPosition(new Rotation2d(Math.toRadians(imu.getAngle())), getWheelPositions(), getPose());
		headingSetPoint = 0;
	}

	/**
	 * tells all motors to go to a certain position, in meters
	 * will move straight to given position if motor encoders are zeroed first
	 */
	public void setPosition(double pos) {
		frontLeftMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS * pos);
		frontRightMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS * pos);
		rearLeftMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS * pos);
		rearRightMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS * pos);
	}

	/**
	 * makes the robot hold its current position
	 */
	public void holdPosition() {
		// takes raw units and getSelectedSensorPosition should give raw units
		frontLeftMotor.set(ControlMode.Position, frontLeftMotor.getSelectedSensorPosition());
		frontRightMotor.set(ControlMode.Position, frontRightMotor.getSelectedSensorPosition());
		rearLeftMotor.set(ControlMode.Position, rearLeftMotor.getSelectedSensorPosition());
		rearRightMotor.set(ControlMode.Position, rearRightMotor.getSelectedSensorPosition());
	}

	public void rotateToGrid() {
		// if (arm.getState().getX() > .4)
		// 	return;
		headingSetPoint = Math.round(imu.getAngle() / 360.0) * 360.0;
	}

	public void rotateToSubstation() {
		// if (arm.getState().getX() > .4)
		// 	return;
		headingSetPoint = Math.round(imu.getAngle() / 360.0) * 360.0 + 180;
	}

	public void rotateFixed(double dir) {
		headingSetPoint = Math.round((imu.getAngle() - dir) / 360.0) * 360.0 + dir;
	}
}

