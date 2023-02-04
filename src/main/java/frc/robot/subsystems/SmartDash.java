package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDash {
    private Double[] displacement;
    private Double[] velocity;
    private Double[] accel;
    private Double[] rotation;

    
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
        displacement = imu.getDisplacement();
        velocity = imu.getVelcoity();
        accel = imu.getAccel() ;
        rotation = imu.getRotation();

        SmartDashboard.putNumber("NavX", displacement[0]);
        SmartDashboard.putNumber("NavY", displacement[1]);
        SmartDashboard.putNumber("NavZ", displacement[2]);

        SmartDashboard.putNumber("NavXVel", velocity[0]);
		SmartDashboard.putNumber("NavYVel", velocity[1]);

        SmartDashboard.putNumber("NavXAccel", accel[0]);
		SmartDashboard.putNumber("NavYAccel", accel[1]);
		SmartDashboard.putNumber("NavZAccel", accel[2]);

        SmartDashboard.putNumber("NavRoll", rotation[0]);
        SmartDashboard.putNumber("NavPitch", rotation[1]);
        SmartDashboard.putNumber("NavHeading", rotation[2]);

		SmartDashboard.putNumber("NavRotationZ", imu.getAngularVelocity());
		SmartDashboard.putBoolean("NavConnected", imu.isNavXConnected());
    }
}
