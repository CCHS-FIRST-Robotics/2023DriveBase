package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

/**
 * Manages the tank drive base
 */
public class TankDrive extends DifferentialDrive  {
	
	public TankDrive(PWMTalonSRX leftMotor, PWMTalonSRX rightMotor) {
		super(leftMotor, rightMotor);
		//TODO Auto-generated constructor stub
	}
}
