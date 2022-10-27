package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

/**
 * Manages the tank drive base
 * 
 */
public class TankDrive extends DifferentialDrive {
	
	// between 0 and 1 - 1 would be full max speed, 0.5 would be half speed, etc
	double maxSpeed = 0.5;

	public TankDrive(PWMTalonSRX leftMotor, PWMTalonSRX rightMotor) {
		super(leftMotor, rightMotor);
		//TODO Auto-generated constructor stub
	}
	
	/**
	 * Drive the robot tank base from controller input 
	 * 
	 * These are doubles on the interval [-1,1], and come from the controller's 
	 * 	analog sticks (circle spinny things)
	 * @param leftAnalogX 
	 * @param leftAnalogY 
	 * @param rightAnalogX 
	 * @param rightAnalogY 
	 */
	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {

		/**
		 * we only care about left Y and right X
		 * left Y is average velocity
		 * right X is velocity difference between wheels
		*/

		// TODO: test multiplying by 2 (to have a larger difference between wheel speeds)
		double x = rightAnalogX; 
		
		double y = leftAnalogY;

		// make sure that both velocities are in [-1, 1]
		double preScaledLeftVel = y + x / 2;
		double preScaledRightVel = y - x / 2;
		double scaleFactor = 1 / Math.max(Math.max(Math.abs(preScaledLeftVel), Math.abs(preScaledRightVel)), 1);
		
		// scale to what the controller asks
		double leftVel = preScaledLeftVel * scaleFactor;
		double rightVel = preScaledRightVel * scaleFactor;
		
		tankDrive(leftVel * maxSpeed, rightVel * maxSpeed);
	}


	/**
	 * The speed bracket controls the multiplier for all the speeds 
	 * So when you change it, lets say, to 1/2 speed, all directions will be
	 * 	at 1/2 speed
	 */
	public void increaseSpeedBracket() {
		maxSpeed = Math.min(0.8, maxSpeed + 0.1);
	}
	public void decreaseSpeedBracket() {
		maxSpeed = Math.max(0.2, maxSpeed - 0.1);
	}
}
