package frc.robot;

import java.lang.Math;

public class Constants {
	final public int xboxControllerPort 		= 0; // the port for the xbox controller
	final public double driveMaxAngularVelocity = 2 * Math.PI;
	final public double driveWheelRadius 		= 0.1; // meters

	public double maxVelocity;

	Constants() {
		maxVelocity = driveMaxAngularVelocity * driveWheelRadius;
		
	}
}
