package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterShuffleboard {
    
    public void pushDashboard(Limelight limelight, IMU imu){
        pushLimelight(limelight);
        pushIMU(imu);
    }

    public void pushLimelight(Limelight limelight) {
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("LimelightArea", limelight.getArea());
    }

    public void pushIMU(IMU imu) {
        SmartDashboard.putNumber("NavX", imu.getDisplacementX());
        SmartDashboard.putNumber("NavY", imu.getDisplacementY());
        SmartDashboard.putNumber("NavZ", imu.getDisplacementZ());

        SmartDashboard.putNumber("NavXVel", imu.getVelocityX());
		SmartDashboard.putNumber("NavYVel", imu.getVelocityY());

        SmartDashboard.putNumber("NavXAccel", imu.getRawAccelX());
		SmartDashboard.putNumber("NavYAccel", imu.getRawAccelY());
		SmartDashboard.putNumber("NavZAccel", imu.getRawAccelZ());

        SmartDashboard.putNumber("NavRoll",imu.getRoll());
        SmartDashboard.putNumber("NavPitch", imu.getPitch());
        SmartDashboard.putNumber("NavHeading", imu.getHeading());

		SmartDashboard.putNumber("NavRotationRateZ", imu.getRate());
		SmartDashboard.putBoolean("NavConnected", imu.isConnected());
    }
}
