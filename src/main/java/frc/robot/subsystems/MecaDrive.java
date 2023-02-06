package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;


import java.lang.Math;

public class MecaDrive extends DriveBase {
    WPI_TalonFX frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

	// Stop mode variables

	// to save the last velocities so the robot can slow down
	double[] slowingDownSpeeds = new double[4];

	// this makes the left and right vel scope include the function that sets
	// the slowing values so the function can use them
	double[] combinedSpeeds = new double[4];

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
		
		// invert motors to make forward the right direction
		frontRightMotor.setInverted(true);
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

		combinedSpeeds = combineSpeeds(leftAnalogX,  leftAnalogY, 
									   rightAnalogX, rightAnalogY);

		switch (currentMode){
			case DEFAULT_MODE:
				 // set the motor speeds as a percent 0-1 (normal)
				frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
				frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
				rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2]);
				rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
				break;
			case STOP_MODE:
				// STOP!!!!! set motors to 0
				// slower stop
				slowingDownSpeeds = slowDown(slowingDownSpeeds);

				frontLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[0]);
				frontRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[1]);
				rearLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[2]);
				rearRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[3]);
				break;
			case DEBUG_MODE:
				// Debug mode (toggle wheels with left stick button)
				
				switch (debugEnabledMotor){
					case 0:
						frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0]);
						break;
					case 1:
						frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
						break;
					case 2:
						rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2]);
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
        // arrays for wheel speeds for each movement direction (percents)
        // 1st is front left, 2nd is front right, 3rd is back left, 4th is back right
        double[] verticalSpeeds = {leftAnalogY, leftAnalogY,
                                   leftAnalogY, leftAnalogY};
        
        // negatives due to wheels going in opposite directions during left or right translation
        double[] horizontalSpeeds = {leftAnalogX, -leftAnalogX,
                                     -leftAnalogX, leftAnalogX};

        // left and right wheels should go different directions to rotate the robot
        double[] rotationSpeeds = {rightAnalogX, -rightAnalogX,
                                   rightAnalogX, -rightAnalogX};
		

		double maxSpeed = Integer.MIN_VALUE;
		/* combined speed could exceed 1 (not good; we cannot run the motors at over 100%)
        we will use the maximum speed to scale all the other speeds to something below 1 */
		for (int i = 0; i < 4; i++) {
			combinedSpeeds[i] = verticalSpeeds[i] + horizontalSpeeds[i] + rotationSpeeds[i];
			if (Math.abs(combinedSpeeds[i]) > maxSpeed) maxSpeed = Math.abs(combinedSpeeds[i]);
		}

		maxSpeed = Math.max(1, maxSpeed); // if the max is under 1, we can ignore (we don't have to do any scaling)

        for (int i = 0; i < 4; i++) {
            // nomralize the speeds and scale by speed multiplier
            combinedSpeeds[i] = (speedMultiplier / maxSpeed) * combinedSpeeds[i];
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
			// velocity needs to be reduced
			newVelocity[i] = inputVelocity[i] / Constants.SLOW_DOWN_FACTOR;

			// input has reached cutoff, now returning 0 speed
			if (Math.abs(inputVelocity[i]) < Constants.SLOW_DOWN_CUTOFF){
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

}
