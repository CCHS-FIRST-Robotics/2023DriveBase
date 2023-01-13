package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

import java.lang.Math;

public class MecaDrive {

	// between 0 and 1
    // 1 would be full max speed, 0.5 would be half speed, etc
	double speedMultiplier = 0.6;

    // scaling down vertical speed because its faster than other speeds
    final double verticalSpeedMultiplier = 0.8;

    // scaling up horinzontal speed because its slower than the other speeds
    final double horizontalSPeedMultiplier = 2.5;

    TalonSRX frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

    // debug mode stops all wheels except one
    boolean debugMode = false;


	/**
	 * Adding modes from TankDrive (it would be better to have a super class that has modes)
	 */

	 String currentMode;
	 final String DEFAULT_MODE = "DEFAULT";
	 final String DEBUG_MODE = "DEBUG";
	 final String PID_TUNING_MODE = "PIDTUNING";
	 final String STOP_MODE = "STOP";

    // the wheel to be activited during debug mode
    int debugEnabledWheel = 0;

	// Stop mode variables

	// to save the last velocities so the robot can slow down
	double[] slowingDownSpeeds = new double[4];

	// this makes the left and right vel scope include the function that sets
	// the slowing values so the function can use them
	double[] combinedSpeeds = new double[4];



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
        frontLeftMotor = new TalonSRX(frontLeftMotorPort);
        frontRightMotor = new TalonSRX(frontRightMotorPort);
        rearLeftMotor = new TalonSRX(rearLeftMotorPort);
        rearRightMotor = new TalonSRX(rearRightMotorPort);
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
	 * Debug mode:
	 * 
	 * 	When you press the left stick down (left stick button) the robot enters
	 * debug mode (bool debugMode), and the robot will only move power one wheel
	 * at a time. Toggle through these wheels by pressing the left stick button 
	 * more (FL -> FR -> BL -> BR) then, after BR, it will leave debug mode and 
	 * go back into all wheel drive
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
		if (rightAnalogX < Constants.ANALOG_DEAD_ZONE){
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

			case STOP_MODE:
				// STOP!!!!! set motors to 0
				// slower stop
				slowingDownSpeeds = slowDown(slowingDownSpeeds);

				frontLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[0] * -1);
				frontRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[1]);
				rearLeftMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[2] * -1);
				rearRightMotor.set(ControlMode.PercentOutput, slowingDownSpeeds[3]);

			case DEBUG_MODE:
				// Debug mode (toggle wheels with left stick button)
				
				switch (debugEnabledWheel){
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

			case PID_TUNING_MODE:
				// nothing yet
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

	/* 
	the speed bracket is the range of speeds that the tank can move and turn in
	 */
	public void increaseSpeedBracket() {
		speedMultiplier = Math.min(1, speedMultiplier + 0.1);
	}
	public void decreaseSpeedBracket() {
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
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
		// Stop mode activated, so now the robot needs to slow down
		// start by saving the last left and right velocities 
		slowingDownSpeeds = combinedSpeeds;
		System.out.println("STOP MODE");
	}
	// for enabling debug and testing different wheels
    public void toggleDebugMode() {
		if (debugMode){
			turnOnDebugMode();
		} else{
			turnOnDefaultMode();
		}
        debugMode = !debugMode;
		//quick fix
        System.out.println("Debug Mode: " + debugMode);
    }

    // cycles through the activated wheels during debug mode
    public void cycleWheelDebugMode() {
        debugEnabledWheel++;
        debugEnabledWheel %= 4;
        System.out.println("Current Wheel: " + debugEnabledWheel);
    }

    // keep implementing modes (stop, debug, defualt) with button presses
}
