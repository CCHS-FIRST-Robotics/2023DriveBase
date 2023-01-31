package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/* 
MANAGES SENSOR DATA
*/
public class Sensors {
	AHRS navx;

	double x, y, z, heading, pitch, zRotationalVelocity, xVelocity, yVelocity, xAccel, yAccel, zAccel; // define navx outputs
	boolean isConnected;

	public Sensors() {
		navx = new AHRS(SPI.Port.kMXP);

	}

	public void getValues() {
		x = navx.getDisplacementX();
		y = navx.getDisplacementY();
		z = navx.getDisplacementZ();
		zRotationalVelocity = navx.getRate();
		xVelocity = navx.getVelocityX();
		yVelocity = navx.getVelocityY();
		pitch = navx.getPitch();
		isConnected = navx.isConnected();
		heading = navx.getFusedHeading();

		// might be better to use the "WorldLinear" methods
		// units are G
		xAccel = navx.getRawAccelX();
		yAccel = navx.getRawAccelY();
		zAccel = navx.getRawAccelZ();
	}

	public void pushShuffleboard() {
		SmartDashboard.putNumber("NavX", x);
        SmartDashboard.putNumber("NavY", y);
        SmartDashboard.putNumber("NavZ", z);
        SmartDashboard.putNumber("NavHead", heading);
		SmartDashboard.putNumber("NavRotationZ", zRotationalVelocity);
		SmartDashboard.putNumber("NavXVel", xVelocity);
		SmartDashboard.putNumber("NavYVel", yVelocity);
		SmartDashboard.putNumber("NavPitch", pitch);
		SmartDashboard.putNumber("NavXAccel", xAccel);
		SmartDashboard.putNumber("NavYAccel", yAccel);
		SmartDashboard.putNumber("NavZAccel", zAccel);
		SmartDashboard.putBoolean("NavConnected", isConnected);
	}
}