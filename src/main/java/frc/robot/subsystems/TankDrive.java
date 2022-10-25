package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

/**
 * Manages the tank drive base
 */
public class TankDrive extends DifferentialDrive {
	
	public TankDrive(PWMTalonSRX leftMotor, PWMTalonSRX rightMotor) {
		super(leftMotor, rightMotor);
		//TODO Auto-generated constructor stub
	}
	
	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {
		// we only care about left Y and right X
		// left Y is average velocity, right X is velocity difference between wheels
		double x = rightAnalogX; // TODO: later test with multiplying by 2
		double y = leftAnalogY;

		double preScaledLeftVel = y + x / 2;
		double preScaledRightVel = y - x / 2;
		// we need to make sure that both velocities are in [-1, 1]
		double scaleFactor = 1 / Math.max(Math.max(Math.abs(preScaledLeftVel), Math.abs(preScaledRightVel)), 1);
		double leftVel = preScaledLeftVel * scaleFactor;
		double rightVel = preScaledRightVel * scaleFactor;
		
		tankDrive(leftVel, rightVel);
	}
}
