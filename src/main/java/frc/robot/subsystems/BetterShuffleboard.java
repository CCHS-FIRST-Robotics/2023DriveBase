package frc.robot.subsystems;
import frc.robot.*;

import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterShuffleboard {

    // shuffleboard tabs
    ShuffleboardTab xboxControllerTuningTab; // used for adjusting things like controller exponent
    GenericEntry leftXExp, leftYExp, rightXExp, rightYExp;

    ShuffleboardTab ArmPIDTab; // used for arm PID tuning
    GenericEntry shoulderP, shoulderI, shoulderD;
    GenericEntry elbowP, elbowI, elbowD;

    ShuffleboardTab debugTab; // used for various debug things
    ShuffleboardTab odomTab; // used for odometry data

    public BetterShuffleboard() {
        /*
         * Xbox Controller tuning
         */
        xboxControllerTuningTab = Shuffleboard.getTab("Tuning");
        // Create widgets for tuning tab
        leftXExp = xboxControllerTuningTab.add("LeftXExp", Constants.LEFT_X_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        leftXExp.setDouble(Constants.LEFT_X_EXPONENT);
        leftYExp = xboxControllerTuningTab.add("LeftYExp", Constants.LEFT_Y_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        leftYExp.setDouble(Constants.LEFT_Y_EXPONENT);
        rightXExp = xboxControllerTuningTab.add("RightXExp", Constants.RIGHT_X_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        rightXExp.setDouble(Constants.RIGHT_X_EXPONENT);
        rightYExp = xboxControllerTuningTab.add("RightYExp", Constants.RIGHT_Y_EXPONENT)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 1, "max", 5))
            .getEntry();
        rightYExp.setDouble(Constants.RIGHT_Y_EXPONENT);

        /*
         * PID tuning
         */

        ArmPIDTab = Shuffleboard.getTab("ArmPID");

        // Create widgets for shoulder PID tab
        shoulderP = ArmPIDTab.add("ShoulderP", Constants.SHOULDER_KP)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        shoulderP.setDouble(Constants.SHOULDER_KP);
        shoulderI = ArmPIDTab.add("ShoulderI", Constants.SHOULDER_KI)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        shoulderI.setDouble(Constants.SHOULDER_KI);
        shoulderD = ArmPIDTab.add("ShoulderD", Constants.SHOULDER_KD)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        shoulderD.setDouble(Constants.SHOULDER_KD);

        // Create widgets for elbow PID tab
        elbowP = ArmPIDTab.add("ElbowP", Constants.ELBOW_KP)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        elbowP.setDouble(Constants.ELBOW_KP);
        elbowI = ArmPIDTab.add("ElbowI", Constants.ELBOW_KI)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        elbowI.setDouble(Constants.ELBOW_KI);
        elbowD = ArmPIDTab.add("ElbowD", Constants.ELBOW_KD)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();
        elbowD.setDouble(Constants.ELBOW_KD);

        /*
         * Others
         */

        debugTab = Shuffleboard.getTab("Debug");
        odomTab = Shuffleboard.getTab("Odometry");
    }

    public void putNumber(String key, double x) {
        SmartDashboard.putNumber(key, x);
    }
    public void putBoolean(String key, boolean x) {
        SmartDashboard.putBoolean(key, x);
    }

    public void pushDashboard(Limelight limelight, IMU imu, ZED zed){
        pushLimelight(limelight);
        pushIMU(imu);
        pushZED(zed);
    }

    public void updateControllerExponents() {
        Constants.LEFT_X_EXPONENT = leftXExp.getDouble(2);
        Constants.LEFT_Y_EXPONENT = leftYExp.getDouble(2);
        Constants.RIGHT_X_EXPONENT = rightXExp.getDouble(2);
        Constants.RIGHT_Y_EXPONENT = rightYExp.getDouble(2);
    }

    public void updatePIDConstants(Arm arm) {
        arm.shoulderP = shoulderP.getDouble(0);
        arm.shoulderI = shoulderI.getDouble(0);
        arm.shoulderD = shoulderD.getDouble(0);

        arm.elbowP = elbowP.getDouble(0);
        arm.elbowI = elbowI.getDouble(0);
        arm.elbowD = elbowD.getDouble(0);

        arm.initControllers();
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

    public void pushZED(ZED zed) {
        SmartDashboard.putNumber("AprilTagX", zed.getAprilTagX());
        SmartDashboard.putNumber("AprilTagY", zed.getAprilTagY());
        SmartDashboard.putNumber("AprilTagZ", zed.getAprilTagZ());
    }
}
