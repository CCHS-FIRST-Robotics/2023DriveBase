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
    GenericEntry odomX, odomY, odomHead; // calculated odometry stuff
    GenericEntry FLVel, FRVel, RLVel, RRVel; // encoder velocities
    // navx data
    GenericEntry NavXVel, NavYVel, NavXAccel, NavYAccel, NavZAccel,
                 NavRoll, NavPitch, NavHeading, NavConnected, NavRotationRateZ;

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
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
        FRVel = odomTab.add("FRVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        FLVel = odomTab.add("FLVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        RRVel = odomTab.add("RRVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        RLVel = odomTab.add("RLVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavXVel = odomTab.add("NavXVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavYVel = odomTab.add("NavYVel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavRotationRateZ = odomTab.add("NavRotationRateZ", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavXAccel = odomTab.add("NavXAccel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavYAccel = odomTab.add("NavYAccel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavZAccel = odomTab.add("NavZAccel", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        NavRoll = odomTab.add("NavRoll", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
        NavPitch = odomTab.add("NavPitch", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
        NavHeading = odomTab.add("NavHeading", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
        // NavConnected = odomTab.add("NavConnected", 0)
        //     .withWidget(BuiltInWidgets.kBooleanBox)
        //     .getEntry();
    }

    public void pushDashboard(Limelight limelight, IMU imu, MecaDrive drive, ZED zed){
        pushLimelight(limelight);
        pushIMU(imu);
        pushZED(zed);
        pushOdom(drive);
    }

    public void updateControllerExponents() {
        Constants.LEFT_X_EXPONENT = leftXExp.getDouble(2);
        Constants.LEFT_Y_EXPONENT = leftYExp.getDouble(2);
        Constants.RIGHT_X_EXPONENT = rightXExp.getDouble(2);
        Constants.RIGHT_Y_EXPONENT = rightYExp.getDouble(2);
    }

    public void pushOdom(MecaDrive drive) {
        odomX.setDouble(drive.getOdomX());
        odomY.setDouble(drive.getOdomY());
        odomHead.setDouble(drive.getOdomHeading());
        FLVel.setDouble(drive.getWheelSpeeds().frontLeftMetersPerSecond);
        FRVel.setDouble(drive.getWheelSpeeds().frontRightMetersPerSecond);
        RLVel.setDouble(drive.getWheelSpeeds().rearLeftMetersPerSecond);
        RRVel.setDouble(drive.getWheelSpeeds().rearRightMetersPerSecond);
    }

    public void pushLimelight(Limelight limelight) {
        SmartDashboard.putNumber("LimelightX", limelight.getX());
        SmartDashboard.putNumber("LimelightY", limelight.getY());
        SmartDashboard.putNumber("LimelightArea", limelight.getArea());
    }

    public void pushIMU(IMU imu) {
        // SmartDashboard.putNumber("NavX", imu.getDisplacementX());
        // SmartDashboard.putNumber("NavY", imu.getDisplacementY());
        // SmartDashboard.putNumber("NavZ", imu.getDisplacementZ());

        NavXVel.setDouble(imu.getVelocityX());
        NavYVel.setDouble(imu.getVelocityY());

        NavXAccel.setDouble(imu.getWorldLinearAccelX());
        NavYAccel.setDouble(imu.getWorldLinearAccelY());
        NavZAccel.setDouble(imu.getWorldLinearAccelZ());

        NavRoll.setDouble(imu.getRoll());
        NavPitch.setDouble(imu.getPitch());
        NavHeading.setDouble(imu.getHeading());

        NavRotationRateZ.setDouble(imu.getRate());
        // NavConnected.setBoolean(imu.isConnected());
    }


    public void pushZED(ZED zed) {
        SmartDashboard.putNumber("AprilTagX", zed.getAprilTagX());
        SmartDashboard.putNumber("AprilTagY", zed.getAprilTagY());
        SmartDashboard.putNumber("AprilTagZ", zed.getAprilTagZ());
    }
}
