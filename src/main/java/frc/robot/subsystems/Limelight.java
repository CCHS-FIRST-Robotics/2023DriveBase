package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight {

    private double x, y, area, forwardDist;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public double getForwardDist(){
        double goalHeightInches           = Constants.TARGET_HIEGHT;
        double limelightLensHeight        = Constants.LIME_HIEGHT;
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0) * ((Math.PI)/(180));
        double limelightMountAngle        = Constants.LIME_ANGLE;

        double angleToGoal = limelightMountAngle + targetOffsetAngle_Vertical;

        forwardDist = (goalHeightInches - limelightLensHeight)/Math.tan(angleToGoal);

        return forwardDist;
    }
    
    public double getHeadingDisplacement() {
        return table.getEntry("tx").getDouble(0.0) * ((Math.PI)/(180));
    }

    // TODO: include comments/documentation for what these vals actually are 
    public double getX() {
            x = table.getEntry("tx").getDouble(0);
            return x;
    }
    
    public double getY() {
            y = table.getEntry("ty").getDouble(0);
            return y;
   }
    
    public double getArea() {
        area = table.getEntry("ta").getDouble(0);
        return area;
    }

}
