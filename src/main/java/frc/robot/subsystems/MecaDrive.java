package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MecaDrive {

	// between 0 and 1
    // 1 would be full max speed, 0.5 would be half speed, etc
	double speedMultiplier = 0.5;

    TalonSRX frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

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

     */
    public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {
        
        // left analog stick controls translation
        // right analog stick controls rotation

        // arrays for wheel speeds (percents)
        // 1st is front left, 2nd is front right, 3rd is back left, 4th is back right
        double[] verticalSpeeds = {leftAnalogY, leftAnalogY,
                                   leftAnalogY, leftAnalogY};
        
        // negatives due to wheels going in opposite directions during left or right translation
        double[] horizontalSpeeds = {-1 * leftAnalogX, leftAnalogX,
                                     leftAnalogX, -1 * leftAnalogX};

        // left and right wheels should go different directions to rotate the robot
        double[] rotationSpeeds = {rightAnalogX, -1 * rightAnalogX,
                                   rightAnalogX, -1 * rightAnalogX};
        
        // so these could exceed 1 (not good; we cannot run the motors at over 100%)
        // we will use the maximum speed to scale all the other speeds to something below 1
        double[] combinedSpeeds = new double[4];
        for (int i = 0; i < 4; i ++)
            combinedSpeeds[i] = verticalSpeeds[i] + horizontalSpeeds[i] + rotationSpeeds[i];

        // find the max of the above speeds so we can check if it is above 1
        double maxSpeed = Integer.MIN_VALUE;
        for (int i = 0; i < 4; i++)
            if (combinedSpeeds[i] > maxSpeed) maxSpeed = combinedSpeeds[i];
        
        maxSpeed = Math.max(1, maxSpeed); // if it is under 1, we can basically ignore it    

        for (int i = 0; i < 4; i++) {
            // scale the speeds
            combinedSpeeds[i] = (1 / maxSpeed) * combinedSpeeds[i];

            // we also need to scale the speeds by the speed multiplier
            combinedSpeeds[i] = combinedSpeeds[i] * speedMultiplier;
        }

        // set the motor speeds
        frontLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[0] * -1);
        frontRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[1]);
        rearLeftMotor.set(ControlMode.PercentOutput, combinedSpeeds[2] * -1);
        rearRightMotor.set(ControlMode.PercentOutput, combinedSpeeds[3]);
    }

	public void increaseSpeedBracket() {
		speedMultiplier = Math.min(0.8, speedMultiplier + 0.1);
	}

	public void decreaseSpeedBracket() {
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
	}
}
