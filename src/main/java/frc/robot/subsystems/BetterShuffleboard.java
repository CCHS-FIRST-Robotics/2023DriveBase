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

    ShuffleboardTab limelightTab; // used for limelight data
    GenericEntry highPostX, lowPostX, highPostY, lowPostY, highPostHeading, lowPostHeading;

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

        // LIMELIGHT
        limelightTab = Shuffleboard.getTab("Limelight");
        
        highPostX = limelightTab.add("highPostX", 0).getEntry();
        lowPostX = limelightTab.add("lowPostX", 0).getEntry();
        lowPostY = limelightTab.add("lowPostY", 0).getEntry();
        highPostY = limelightTab.add("highPostY", 0).getEntry();
        highPostHeading = limelightTab.add("highPostHeading", 0).getEntry();
        lowPostHeading = limelightTab.add("lowPostHeading", 0).getEntry();

        debugTab = Shuffleboard.getTab("Debug");
        odomTab = Shuffleboard.getTab("Odometry");
    }

    public void pushDashboard(Limelight limelight, IMU imu){
        pushLimelight(limelight);
        pushIMU(imu);
    }

    public void updateControllerExponents() {
        Constants.LEFT_X_EXPONENT = leftXExp.getDouble(2);
        Constants.LEFT_Y_EXPONENT = leftYExp.getDouble(2);
        Constants.RIGHT_X_EXPONENT = rightXExp.getDouble(2);
        Constants.RIGHT_Y_EXPONENT = rightYExp.getDouble(2);
    }

    public void pushLimelight(Limelight limelight) {
        highPostX.setDouble(limelight.getX(Constants.TALL_PIPE_NUM));
        highPostY.setDouble(limelight.getY(Constants.TALL_PIPE_NUM));
        highPostHeading.setDouble(limelight.getHeadingDisplacement(Constants.TALL_PIPE_NUM));
        
        lowPostX.setDouble(limelight.getX(Constants.SHORT_PIPE_NUM));
        lowPostY.setDouble(limelight.getY(Constants.SHORT_PIPE_NUM));
        lowPostHeading.setDouble(limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM));
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
