package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


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

    // the wheel to be activited during debug mode
    int debugEnabledWheel = 0;


    /**
     * Constructor for MecaDrive Class
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
        System.out.println("Hello World");
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
        double[] combinedSpeeds = new double[4];
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

        // set the motor speeds (normal)
        if(!debugMode) {
            frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0] * -1);
            frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
            rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2] * -1);
            rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
        } 
		
		// Debug mode (toggle wheels with left stick button)
		else {
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
        }
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

	// for enabling debug and testing different wheels
    public void toggleDebugMode() {
        debugMode = !debugMode;
        System.out.println("Debug Mode: " + debugMode);
    }

    // cycles through the activated wheels during debug mode
    public void cycleWheelDebugMode() {
        debugEnabledWheel++;
        debugEnabledWheel %= 4;
        System.out.println("Current Wheel: " + debugEnabledWheel);
    }

    
}
