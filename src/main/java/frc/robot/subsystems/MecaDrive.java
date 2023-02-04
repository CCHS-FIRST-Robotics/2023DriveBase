package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

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

public class MecaDrive extends DriveBase {
    // scaling down vertical speed because its faster than other speeds
    private final double verticalSpeedMultiplier = 0.8;

    // scaling up horinzontal speed because its slower than the other speeds
    final double horizontalSPeedMultiplier = 0.8;

    private WPI_TalonFX frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

	// Stop mode variables

	// to save the last velocities so the robot can slow down
	private double[] slowingDownSpeeds = new double[4];

	// this makes the left and right vel scope include the function that sets
	// the slowing values so the function can use them
	private double[] combinedSpeeds = new double[4];

	MecaSubsystem subsystem;

	// input curving for better fine control
	public static double LEFT_Y_EXPONENT = 2;
	public static double LEFT_X_EXPONENT = 2;
	public static double RIGHT_X_EXPONENT = 2;

    /**
     * Constructor for Mecanum Drive Class
     * 
     * @param frontLeftMotorPort
     * @param rearLeftMotorPort
     * @param frontRightMotorPort
     * @param rearRightMotorPort
     */
    public MecaDrive(int frontLeftMotorPort, int frontRightMotorPort,
                     int rearLeftMotorPort, int rearRightMotorPort) {
        frontLeftMotor = new WPI_TalonFX(frontLeftMotorPort);
        frontRightMotor = new WPI_TalonFX(frontRightMotorPort);
        rearLeftMotor = new WPI_TalonFX(rearLeftMotorPort);
        rearRightMotor = new WPI_TalonFX(rearRightMotorPort);

		subsystem = new MecaSubsystem(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		
		// invert motors to make forward the right direction
		frontLeftMotor.setInverted(true);
		frontRightMotor.setInverted(true);
		rearLeftMotor.setInverted(true);
		rearRightMotor.setInverted(true);
    } 

    /**
     * drive the robot using controller inputs
     * 
     * Left analog stick controls translation
     * Right analog stick controls rotation (move it right -> rotate clockwise, move it left -> rotate counterclockwise)
     * 
     * positive is right/up
     * 
     * @param leftAnalogX the x position of the left analog stick; range: [-1, 1]
     * @param leftAnalogY the y position of the left analog stick; range: [-1, 1]
     * @param rightAnalogX the x position of the right analog stick; range: [-1, 1]
     * @param rightAnalogY the y position of the right analog stick; range: [-1, 1]
	 * 
	 * In Debug Mode, the robot will only power one wheel at a time.
	 * Toggle through these wheels by pressing A.
	 * Wheel Order: FL -> FR -> BL -> BR -> FL -> ...
	 * 
     */
    public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {
        
        // left analog stick controls translation
        // right analog stick controls rotation
		

		 /**
		 * Add deadbands (stop all movement when input is under a certain amount)
		 * This allows you to go online in one direction more easily because it 
		 * prevents slight (unintentional) inputs in the perpendicular direction
		 */
		
		if (Math.abs(leftAnalogX) < Constants.ANALOG_DEAD_ZONE) {
			leftAnalogX = 0;
		}
		else {
			leftAnalogX = Math.pow(Math.abs(leftAnalogX), LEFT_X_EXPONENT) * Math.signum(leftAnalogX);
		}
		if (Math.abs(rightAnalogX) < Constants.ANALOG_DEAD_ZONE){
			rightAnalogX = 0;
		}
		else {
			rightAnalogX = Math.pow(Math.abs(rightAnalogX), RIGHT_X_EXPONENT) * Math.signum(rightAnalogX);
		}

		combinedSpeeds = combineSpeeds(leftAnalogX,  leftAnalogY, 
									   rightAnalogX, rightAnalogY);

		switch (currentMode){
			case DEFAULT_MODE:
				 // set the motor speeds (normal)
				frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0] * -1);
				frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
				rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2] * -1);
				rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
				break;
			case STOP_MODE:
				// STOP!!!!! set motors to 0
				// slower stop
				slowingDownSpeeds = slowDown(slowingDownSpeeds);

				frontLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[0] * -1);
				frontRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[1]);
				rearLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[2] * -1);
				rearRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[3]);
				break;
			case DEBUG_MODE:
				// Debug mode (toggle wheels with left stick button)
				
				switch (debugEnabledMotor){
					case 0:
						frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0] * -1);
						break;
					case 1:
						frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
						break;
					case 2:
						rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2] * -1);
						break;
					case 3:
						rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
						break;	
        		}

				break;
			case PID_TUNING_MODE:
				// nothing yet
				break;
		}
    }

	/**
	 * Drive method for WPI_Lib motor objects
	 * 
	 * @param leftAnalogX
	 * @param leftAnalogY
	 * @param rightAnalogX
	 */

	public void drive(double leftAnalogX, double leftAnalogY, double rightAnalogX) {
		subsystem.drive(leftAnalogX, leftAnalogY, rightAnalogX);
	}
	

	@Override
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

	@Override
	public void AButtonPressed() {
		switch(currentMode) {
			case DEBUG_MODE:
				cycleMotor();
				break;
			default:
				break;
		}
	}

	@Override
	public void BButtonPressed() {
		switch(currentMode) {
			case DEBUG_MODE:
				printActiveMotorDebugMode();
				break;
			default:
				break;
		}
	}

	@Override
	public void leftBumperPressed() {
		decreaseSpeedBracket();
	}

	@Override
	public void rightBumperPressed() {
		increaseSpeedBracket();
	}

	/**
	 * 
	 * Process the controller input into speeds for mecanum wheels
	 * 
	 * @param leftAnalogX
	 * @param leftAnalogY
	 * @param rightAnalogX
	 * @param rightAnalogY
	 * @return array of all processed speeds {front left, front right, back left, back right}
	 */
	private double[] combineSpeeds(double leftAnalogX,  double leftAnalogY,
								   double rightAnalogX, double rightAnalogY){

		leftAnalogY *= verticalSpeedMultiplier;
        leftAnalogX *= horizontalSPeedMultiplier;
       
        // arrays for wheel speeds (percents)
        // 1st is front left, 2nd is front right, 3rd is back left, 4th is back right
        double[] verticalSpeeds = {leftAnalogY, leftAnalogY,
                                   leftAnalogY, leftAnalogY};
        
        // negatives due to wheels going in opposite directions during left or right translation
        double[] horizontalSpeeds = {leftAnalogX, -1 * leftAnalogX,
                                     -1 * leftAnalogX, leftAnalogX};

        // left and right wheels should go different directions to rotate the robot
        double[] rotationSpeeds = {rightAnalogX, -1 * rightAnalogX,
                                   rightAnalogX, -1 * rightAnalogX};
        
        // so these could exceed 1 (not good; we cannot run the motors at over 100%)
        // we will use the maximum speed to scale all the other speeds to something below 1
        for (int i = 0; i < 4; i ++)
            combinedSpeeds[i] = verticalSpeeds[i] + horizontalSpeeds[i] + rotationSpeeds[i];

        // find the max of the above speeds so we can check if it is above 1
        double maxSpeed = Integer.MIN_VALUE;

        for (int i = 0; i < 4; i++)
            if (Math.abs(combinedSpeeds[i]) > maxSpeed) maxSpeed = Math.abs(combinedSpeeds[i]);
        
        maxSpeed = Math.max(1, maxSpeed); // if it is under 1, we can basically ignore it    

        for (int i = 0; i < 4; i++) {
            // scale the speeds
            combinedSpeeds[i] = (1 / maxSpeed) * combinedSpeeds[i];

            // we also need to scale the speeds by the speed multiplier
            combinedSpeeds[i] = combinedSpeeds[i] * speedMultiplier;
        }

		return combinedSpeeds;
	}

	/**
	 * This will return a value lower than the input, and it is used to slow 
	 * down the motors during stop mode
	 * 
	 * @param inputVelocity between -1 and 1 (double)
	 * @return the new value (lower)
	 */
	private double[] slowDown(double[] inputVelocity){
		// input is between -1 and 1
		double[] newVelocity = new double[4];

		for (int i = 0; i < 4; i++){
			if (Math.abs(inputVelocity[i]) > Constants.SLOW_DOWN_CUTOFF){
				// velocity still needs to be reduced (magnatude is above cutoff)
				newVelocity[i] = inputVelocity[i] / Constants.SLOW_DOWN_FACTOR;
			} else {
				// input has reached cutoff, now returning 0 speed
				newVelocity[i] = 0;
			}
		}
		return newVelocity;
	}

	@Override
	public void turnOnStopMode() {
		if(currentMode == Mode.STOP_MODE) return;
		currentMode = Mode.STOP_MODE;
		// Stop mode activated, so now the robot needs to slow down
		// start by saving the last left and right velocities 
		slowingDownSpeeds = combinedSpeeds;
		System.out.println("STOP MODE");
	}

	// prints the number of the currently activated motor during debug mode
	public void printActiveMotorDebugMode() {
		System.out.println("Current Motor: " + debugEnabledMotor);
	}

    // cycles through the activated motors during debug mode
    public void cycleMotor() {
        debugEnabledMotor++;
        debugEnabledMotor %= 4;
        System.out.println("Current Motor: " + debugEnabledMotor);
    }

	/**
	 * 
	 * MecaSubsystem class handles all drive functions outside of manual driving
	 * 
	 */
	private class MecaSubsystem extends SubsystemBase {

		// TODO: create better modules for teleop and autonomous driving / odometry

		// TODO: determine whether we need to update navx

		// The gyro sensor
			// TODO: USE SHUFFLEBOARD/SENSORS.JAVA NOT NAVX IN THIS FILE
		final AHRS navx = new AHRS(Port.kMXP);


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

		MecaSubsystem(WPI_TalonFX frontLeftMotor, WPI_TalonFX rearLeftMotor, WPI_TalonFX frontRightMotor, WPI_TalonFX rearRightMotor) {

			// Initializing motors, drive
			this.frontLeftMotor = frontLeftMotor;
			this.rearLeftMotor = rearLeftMotor;
			this.frontRightMotor = frontRightMotor;
			this.rearRightMotor = rearRightMotor;
			this.rearLeftMotor.setInverted(true);
			this.frontLeftMotor.setInverted(true);
			mDrive = new MecanumDrive(this.frontLeftMotor, this.rearLeftMotor, this.frontRightMotor, this.rearRightMotor);
			
			// Initializing Falcon sensors
			flFalconSensor = new TalonFXSensorCollection(this.frontLeftMotor);
			rlFalconSensor = new TalonFXSensorCollection(this.rearLeftMotor);
			frFalconSensor = new TalonFXSensorCollection(this.frontRightMotor);
			rrFalconSensor = new TalonFXSensorCollection(this.rearRightMotor);

			// Odometry: !!secondary constructor takes initialPose argument
			mOdom = new MecanumDriveOdometry(Constants.MECANUM_KINEMATICS, new Rotation2d(Math.toRadians(navx.getAngle())), getWheelPositions());
		}
		
		void drive(double speedX, double speedY, double rotateSpeed) {

			//TODO: I think we'd only want this for teleop so maybe we throw it in Robot.java since all drive systems use it?
			if(Math.abs(speedX) < Constants.ANALOG_CROSS_DEADZONE) {
				speedX = 0;
			}
			if(Math.abs(speedY) < Constants.ANALOG_CROSS_DEADZONE) {
				speedY = 0;
			}
			
			// method defines Y as left/right and X as forward/backward - left/forward are positive
			// TODO: check to make sure xbox controller defines left as negative (which is why we reverse the sign here)
			mDrive.driveCartesian(speedY, -speedX, rotateSpeed);
		}
		
		@Override
		public void periodic() {
			// Update the odometry in the periodic block
			mOdom.update(new Rotation2d(Math.toRadians(navx.getAngle())), getWheelPositions());
		}

		/**
		 * Returns the total distances measured by each motor
		 * 
		 * @return wheel positions
		 */
		MecanumDriveWheelPositions getWheelPositions() {

			// TODO: determine whether should use absolute position or jsut position

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
				new Rotation2d(Math.toRadians(navx.getAngle())), getWheelPositions(), pose);
		}

		/**
		 * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
		 *
		 * @param maxOutput the maximum output to which the drive will be constrained
		 */
		public void setMaxOutput(double maxOutput) {
			mDrive.setMaxOutput(maxOutput);
		}

		/** Zeroes the heading of the robot. */
		public void zeroHeading() {
			navx.reset();
		}

		/**
		 * Returns the heading of the robot.
		 *
		 * @return the robot's heading in degrees, from -180 to 180
		 */
		public double getHeading() {
			return navx.getYaw();
		}

		/**
		 * Returns the turn rate of the robot.
		 *
		 * @return The turn rate of the robot, in degrees per second
		 */
		public double getTurnRate() {
			return -navx.getVelocityZ();
		}

		
		
		// TODO: create structure for odometry

	}

}

