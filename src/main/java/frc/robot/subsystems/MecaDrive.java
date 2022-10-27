package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class MecaDrive extends MecanumDrive {

	// between 0 and 1 - 1 would be full max speed, 0.5 would be half speed, etc
	double speedMultiplier = 0.5;

    public MecaDrive(PWMTalonSRX frontLeftMotor, PWMTalonSRX rearLeftMotor,
                    PWMTalonSRX frontRightMotor, PWMTalonSRX rearRightMotor) {
       super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
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
        // right analog stick cntrols
        driveCartesian(leftAnalogY * speedMultiplier, leftAnalogX * speedMultiplier, rightAnalogX * speedMultiplier);
    }

	public void increaseSpeedBracket() {
		speedMultiplier = Math.min(0.8, speedMultiplier + 0.1);
	}

	public void decreaseSpeedBracket() {
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
	}
}
