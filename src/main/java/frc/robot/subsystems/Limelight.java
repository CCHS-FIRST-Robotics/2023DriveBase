package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight {

    private double x, y, area, forwardDist;
    private int pipeNum;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public double getForwardDist(int pipeChoice){
        changePipeline(pipeChoice);

        double goalHeight = 0;

        if (pipeChoice == 0) {
            goalHeight = Constants.SHORT_TARGET_HEIGHT;
        } else if (pipeChoice == 1) {
            goalHeight = Constants.TALL_TARGET_HEIGHT;
        }

        double limelightLensHeight        = Constants.LIME_HEIGHT;
        double limelightMountAngle        = Constants.LIME_ANGLE;
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);

        double angleToGoal = limelightMountAngle + targetOffsetAngle_Vertical;
        angleToGoal = Math.toRadians(angleToGoal);

        forwardDist = (goalHeight - limelightLensHeight)/Math.tan(angleToGoal);

        return forwardDist;
    }
    
    public double getHeadingDisplacement(int pipeChoice) {
        changePipeline(pipeChoice);
        return table.getEntry("tx").getDouble(0.0) * ((Math.PI)/(180));
    }

    // TODO: include comments/documentation for what these vals actually are 
    public double getX(int pipeChoice) {
        changePipeline(pipeChoice);
        x = table.getEntry("tx").getDouble(0);
        return x;
    }
    
    public double getY(int pipeChoice) {
        changePipeline(pipeChoice);
        y = table.getEntry("ty").getDouble(0);
        return y;
   }
    
    public double getArea(int pipeChoice) {
        changePipeline(pipeChoice);
        area = table.getEntry("ta").getDouble(0);
        return area;
    }

    public void changePipeline(int newPipeNum){
        table.getEntry("pipeline").setInteger(newPipeNum);

        pipeNum = newPipeNum;
    }

    public int getPipeline(){
        return pipeNum;
    }

}
