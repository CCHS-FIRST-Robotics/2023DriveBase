package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterShuffleboard {

    // shuffleboard tabs
    ShuffleboardTab tuningTab; // used for adjusting things like controller exponent
    GenericEntry leftXExp, leftYExp, rightXExp, rightYExp;
    ShuffleboardTab debugTab; // used for various debug things
    ShuffleboardTab odomTab; // used for odometry data
    GenericEntry odomX, odomY, odomHead;

    public BetterShuffleboard() {
        tuningTab = Shuffleboard.getTab("Tuning");
        // create widgets for tuning tab
        leftXExp = tuningTab.add("LeftXExp", Constants.LEFT_X_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        leftXExp.setDouble(Constants.LEFT_X_EXPONENT);
        leftYExp = tuningTab.add("LeftYExp", Constants.LEFT_Y_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        leftYExp.setDouble(Constants.LEFT_Y_EXPONENT);
        rightXExp = tuningTab.add("RightXExp", Constants.RIGHT_X_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        rightXExp.setDouble(Constants.RIGHT_X_EXPONENT);
        rightYExp = tuningTab.add("RightYExp", Constants.RIGHT_Y_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        rightYExp.setDouble(Constants.RIGHT_Y_EXPONENT);

        debugTab = Shuffleboard.getTab("Debug");
        odomTab = Shuffleboard.getTab("Odometry");
        odomX = odomTab.add("OdomX", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("visible time", 20))
            .getEntry();
        odomY = odomTab.add("OdomY", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("visible time", 20))
            .getEntry();
        odomHead = odomTab.add("OdomHead", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 360))
            .getEntry();
    }

    public void pushDashboard(Limelight limelight, IMU imu, MecaSubsystem drive){
        pushLimelight(limelight);
        pushIMU(imu);
        pushOdom(drive);
    }

    public void updateControllerExponents() {
        Constants.LEFT_X_EXPONENT = leftXExp.getDouble(2);
        Constants.LEFT_Y_EXPONENT = leftYExp.getDouble(2);
        Constants.RIGHT_X_EXPONENT = rightXExp.getDouble(2);
        Constants.RIGHT_Y_EXPONENT = rightYExp.getDouble(2);
    }

    public void pushOdom(MecaSubsystem drive) {
        odomX.setDouble(drive.getOdomX());
        odomY.setDouble(drive.getOdomY());
        odomHead.setDouble(drive.getOdomHeading());
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
