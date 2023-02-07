package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight {

    double x, y, area, forwardDist;
    NetworkTableEntry tx, ty, ta;
    NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

    }

    public void smartDash() {
        x    = tx.getDouble(0);
        y    = ty.getDouble(0);
        area = ta.getDouble(0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    public double getForwardDist(){
        double goalHeightInches           = Constants.TARGET_HIEGHT;
        double limelightLensHeight        = Constants.LIME_HIEGHT;
        double targetOffsetAngle_Vertical = ty.getDouble(0.0) * ((Math.PI)/(180));
        double limelightMountAngle        = Constants.LIME_ANGLE;

        double angleToGoal = limelightMountAngle + targetOffsetAngle_Vertical;

        forwardDist = (goalHeightInches - limelightLensHeight)/Math.tan(angleToGoal);

        return forwardDist;
    }

}
