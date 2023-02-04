package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/* 
MANAGES IMU DATA
*/
public class IMU {
	AHRS navx;

	private double x, y, z, heading, pitch, roll, zRotationalVelocity, xVelocity, yVelocity, xAccel, yAccel, zAccel; // define navx outputs
	private boolean isConnected;

	public IMU() {
		navx = new AHRS(SPI.Port.kMXP);
	}

	public void updateValues() {
		x = navx.getDisplacementX();
		y = navx.getDisplacementY();
		z = navx.getDisplacementZ();

		zRotationalVelocity = navx.getRate();

		xVelocity = navx.getVelocityX();
		yVelocity = navx.getVelocityY();
		
		isConnected = navx.isConnected();
		
		pitch = navx.getPitch();
		roll = navx.getRoll();
		// only use fused if magnetometer is working properly
		heading = navx.getFusedHeading();
		if (navx.isMagneticDisturbance() || !navx.isMagnetometerCalibrated()) navx.getYaw();

		// might be better to use the "WorldLinear" methods
		// units are G
		xAccel = navx.getRawAccelX();
		yAccel = navx.getRawAccelY();
		zAccel = navx.getRawAccelZ();
	}

	public Double[] getDisplacement() {
		var displacement = new Double[3];
		displacement[0] = x;
		displacement[1] = y;
		displacement[2] = z;
		return displacement;
	}

	public Double[] getVelcoity() {
		var velcoity = new Double[2];
		velcoity[0] = xAccel;
		velcoity[1] = yAccel;
		return velcoity;
	}

	public Double[] getAccel() {
		var accel = new Double[3];
		accel[0] = xAccel;
		accel[1] = yAccel;
		accel[2] = zAccel;
		return accel;
	}

	public Double[] getRotation() {
		var rotation = new Double[3];
		rotation[0] = roll;
		rotation[1] = pitch;
		rotation[2] = heading;
		return rotation;
	}

	public double getAngularVelocity() {
		return zRotationalVelocity;
	}

	public boolean isNavXConnected() {
		return isConnected;
	}

}