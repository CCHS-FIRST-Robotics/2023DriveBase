package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

import javax.lang.model.util.ElementScanner14;

public class Limelight {

    private double x, y, area, forwardDistance;
    private double pipeNum;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public double getForwardDistance(int pipeChoice) {
        changePipeline(pipeChoice);

        double goalHeight = 0;

        if (pipeChoice == 0) {
            goalHeight = Constants.SHORT_TARGET_HEIGHT;
        } else if (pipeChoice == 1) {
            goalHeight = Constants.TALL_TARGET_HEIGHT;
        }

        double limelightLensHeight       = Constants.LIME_HEIGHT;
        double limelightMountAngle       = Constants.LIME_ANGLE;
        double targetOffsetAngleVertical  = table.getEntry("ty").getDouble(0.0) * ((Math.PI)/(180));

        double angleToGoal = limelightMountAngle + targetOffsetAngleVertical;

        forwardDistance = (goalHeight - limelightLensHeight)/Math.tan(angleToGoal);

        return forwardDistance;
    }
    
    public double getHeadingDisplacement(int pipeChoice) {
        changePipeline(pipeChoice);
        return table.getEntry("tx").getDouble(0.0) * ((Math.PI)/(180));
    }

    // TODO: include comments/documentation for what these vals actually are 
    // gives x angle
    public double getX(int pipeChoice) {
        changePipeline(pipeChoice);
        x = table.getEntry("tx").getDouble(0);
        return x;
    }
    
    // gives y angle
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

    // double testPipe = 0;
    public void changePipeline(double newPipeNum) {

        table.getEntry("pipeline").setNumber(newPipeNum);
        
        double test = table.getEntry("getpipe").getDouble(0);

        if (test != newPipeNum)
        {
            System.out.println("Current pipeline: " + test);
            System.out.println("Expected pipeline: " + newPipeNum);
            System.out.println("-----------------");
        }
        else
            pipeNum = newPipeNum;
        
    }

    public int getPipeline(){
        return pipeNum;
    }

}
