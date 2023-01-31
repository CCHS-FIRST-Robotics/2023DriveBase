package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.Math;

public class MecaDrive extends DriveBase {
    // scaling down vertical speed because its faster than other speeds
    private final double verticalSpeedMultiplier = 0.8;

    // scaling up horinzontal speed because its slower than the other speeds
    private final double horizontalSPeedMultiplier = 0.8;

    private WPI_TalonFX frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

	// Stop mode variables

	// to save the last velocities so the robot can slow down
	private double[] slowingDownSpeeds = new double[4];

	// this makes the left and right vel scope include the function that sets
	// the slowing values so the function can use them
	private double[] combinedSpeeds = new double[4];

	private MecanumDrive mDrive;


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

		mDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
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
		 * Add deadzone (stop all movement when input is under a certain amount)
		 * Compare joystick distance from normal position (0)
		 */
		
		// Left analog joystick distance from origin from pythagoreas 
		double leftJoystickDistance = Math.sqrt(Math.pow(leftAnalogX, 2) +
									 			Math.pow(leftAnalogY, 2));
		if (leftJoystickDistance < Constants.ANALOG_DEAD_ZONE) {
			// joystick is within the deadzone, so set to 0
			leftAnalogX = 0;
			leftAnalogY = 0;
		}
		if (Math.abs(rightAnalogX) < Constants.ANALOG_DEAD_ZONE){
			rightAnalogX = 0;
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
	 * WPI_LIB Drive Function
	 * 
	 * @param x empty argument meant for differentiating the two drive mthods. Does nothing, input any integer
	 * 
	 */
	public void driveWPI(double leftAnalogX, double leftAnalogY, double rightAnalogX) {
		mDrive.driveCartesian(leftAnalogY, rightAnalogX, rightAnalogX);
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
		}
	}

	@Override
	public void BButtonPressed() {
		switch(currentMode) {
			case DEBUG_MODE:
				printActiveMotorDebugMode();
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
        double[] horizontalSpeeds = {-1 * leftAnalogX, leftAnalogX,
                                     leftAnalogX, -1 * leftAnalogX};

        // left and right wheels should go different directions to rotate the robot
        double[] rotationSpeeds = {-1 * rightAnalogX, rightAnalogX,
                                   -1 * rightAnalogX, rightAnalogX};
        
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
		if(currentMode.equals(STOP_MODE)) return;
		currentMode = STOP_MODE;
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

}


