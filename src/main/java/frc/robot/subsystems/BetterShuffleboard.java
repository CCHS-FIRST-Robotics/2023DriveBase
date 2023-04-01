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
    GenericEntry odomX, odomY, odomHead; // calculated odometry stuff
    GenericEntry FLVel, FRVel, RLVel, RRVel; // encoder velocities

    ShuffleboardTab matchTab; // used for in-match data
    GenericEntry motorLimited, limitsActive, clawArmed;
    
    // navx data
    GenericEntry NavXVel, NavYVel, NavXAccel, NavYAccel, NavZAccel,
                 NavRoll, NavPitch, NavHeading, NavFused, NavCompass, NavConnected, NavRotationRateZ;

    ShuffleboardTab limelightTab; // used for limelight data
    GenericEntry highPostX, lowPostX, highPostY, lowPostY, highPostHeading, lowPostHeading, 
                 highForwardDistance, lowForwardDistance;

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

        // LIMELIGHT
        limelightTab = Shuffleboard.getTab("Limelight");
        
        highPostX = limelightTab.add("highPostX", 0).getEntry();
        lowPostX = limelightTab.add("lowPostX", 0).getEntry();
        lowPostY = limelightTab.add("lowPostY", 0).getEntry();
        highPostY = limelightTab.add("highPostY", 0).getEntry();
        highPostHeading = limelightTab.add("highPostHeading", 0).getEntry();
        lowPostHeading = limelightTab.add("lowPostHeading", 0).getEntry();
        highForwardDistance = limelightTab.add("highForwardDist", 0).getEntry();
        lowForwardDistance = limelightTab.add("lowForwardDist", 0).getEntry();

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
         * IN-MATCH DATA
         */ 

        matchTab = Shuffleboard.getTab("Match Data");
        motorLimited = matchTab.add("IS THE ARM PAST A LIMIT?", false)
            .withPosition(5, 3)
            .withSize(5, 1)
            .getEntry();
        limitsActive = matchTab.add("ARE MOTOR LIMITS ON?", true)
            .withPosition(5, 0)
            .withSize(5, 3)
            .getEntry();
        clawArmed = matchTab.add("IS THE CLAW ARMED?", false)
            .withPosition(0, 0)
            .withSize(5, 4)
            .getEntry();

        /*
        * Others
        */

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
        NavFused = odomTab.add("NavFused", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();
        NavCompass = odomTab.add("NavCompass", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 360))
            .getEntry();
        // NavConnected = odomTab.add("NavConnected", 0)
        //     .withWidget(BuiltInWidgets.kBooleanBox)
        //     .getEntry();
    }

    public void putNumber(String key, double x) {
        SmartDashboard.putNumber(key, x);
    }
    public void putBoolean(String key, boolean x) {
        SmartDashboard.putBoolean(key, x);
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

    public void updatePIDConstants(Arm arm) {
        arm.shoulderP = shoulderP.getDouble(0);
        arm.shoulderI = shoulderI.getDouble(0);
        arm.shoulderD = shoulderD.getDouble(0);

        arm.elbowP = elbowP.getDouble(0);
        arm.elbowI = elbowI.getDouble(0);
        arm.elbowD = elbowD.getDouble(0);
    }

    public void pushMatchData(boolean areLimitsActive, boolean motorsStopped, boolean isClawArmed) {
        motorLimited.setBoolean(motorsStopped);
        limitsActive.setBoolean(areLimitsActive);
        clawArmed.setBoolean(isClawArmed);
    }

    public void pushOdom(MecaDrive drive) {
        odomX.setDouble(drive.getPoseX());
        odomY.setDouble(drive.getPoseY());
        odomHead.setDouble(drive.getPoseHeading());
        FLVel.setDouble(drive.getWheelSpeeds().frontLeftMetersPerSecond);
        FRVel.setDouble(drive.getWheelSpeeds().frontRightMetersPerSecond);
        RLVel.setDouble(drive.getWheelSpeeds().rearLeftMetersPerSecond);
        RRVel.setDouble(drive.getWheelSpeeds().rearRightMetersPerSecond);
    }

    public void pushLimelight(Limelight limelight) {
        highPostX.setDouble(limelight.getX(Constants.TALL_PIPE_NUM));
        highPostY.setDouble(limelight.getY(Constants.TALL_PIPE_NUM));
        highPostHeading.setDouble(limelight.getHeadingDisplacement(Constants.TALL_PIPE_NUM));
        highForwardDistance.setDouble(limelight.getForwardDistance(Constants.TALL_PIPE_NUM));
        
        lowPostX.setDouble(limelight.getX(Constants.SHORT_PIPE_NUM));
        lowPostY.setDouble(limelight.getY(Constants.SHORT_PIPE_NUM));
        lowPostHeading.setDouble(limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM));
        lowForwardDistance.setDouble(limelight.getForwardDistance(Constants.SHORT_PIPE_NUM));
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
        NavFused.setDouble(imu.getFusedHeading());
        NavCompass.setDouble(imu.getCompassHeading());

        NavRotationRateZ.setDouble(imu.getRate());
        // NavConnected.setBoolean(imu.isConnected());
    }


    public void pushZED(ZED zed) {
        SmartDashboard.putNumber("AprilTagX", zed.getAprilTagX());
        SmartDashboard.putNumber("AprilTagY", zed.getAprilTagY());
        SmartDashboard.putNumber("AprilTagZ", zed.getAprilTagZ());
    }
}
