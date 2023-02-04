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
		if (navx.isMagneticDisturbance() || !navx.isMagnetometerCalibrated()) heading = navx.getYaw();

		// might be better to use the "WorldLinear" methods - removes the effect of gravity
		// units are G
		xAccel = navx.getRawAccelX();
		yAccel = navx.getRawAccelY();
		zAccel = navx.getRawAccelZ();
	}
