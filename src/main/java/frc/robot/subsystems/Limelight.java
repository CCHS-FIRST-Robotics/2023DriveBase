package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

import javax.lang.model.util.ElementScanner14;

public class Limelight {

    // x and y are angles from center of lense tocenter of reflective tape in degrees
    // area is percentage of screen the tape takes up
    // forward distance is meters
    private double x, y, area, forwardDistance;
    // pipeline number (0 is low, 1 is high)
    private double pipeNum;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double getForwardDistance(int pipeChoice) {
        changePipeline(pipeChoice);

        double goalHeight = 0;

        if (pipeChoice == 0)
            goalHeight = Constants.SHORT_TARGET_HEIGHT;
        if (pipeChoice == 1)
            goalHeight = Constants.TALL_TARGET_HEIGHT;

        double h = goalHeight - Constants.LIME_HEIGHT;
        double angleToGoalY = getPitch(pipeChoice) * Math.PI / 180;
        forwardDistance = h / Math.tan(angleToGoalY);

        return forwardDistance;
    }

    public double getPitch(int pipeChoice) {
        changePipeline(pipeChoice);
        double angleToGoalY = Constants.LIME_ANGLE_Y + table.getEntry("ty").getDouble(0);
        return angleToGoalY;
    }

    public double getYaw(int pipeChoice) {
        changePipeline(pipeChoice);
        double angleToGoalX = Constants.LIME_ANGLE_X + table.getEntry("tx").getDouble(0);
        return angleToGoalX;
    }
    
    public double getHeadingDisplacement(int pipeChoice) {
        changePipeline(pipeChoice);
        return table.getEntry("tx").getDouble(0.0) * Math.PI / 180;
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

    // double testPipe = 0;
    public void changePipeline(double newPipeNum) {

        table.getEntry("pipeline").setNumber(newPipeNum);
        
        // double test = table.getEntry("getpipe").getDouble(0);

        // if (test != newPipeNum)
        // {
        //     System.out.println("Current pipeline: " + test);
        //     System.out.println("Expected pipeline: " + newPipeNum);
        //     System.out.println("-----------------");
        // }
        // else
        //     pipeNum = newPipeNum;
        //     // testPipe = (testPipe + 1) % 2;
        //     //colin says: oui oui baguette is yummy

        pipeNum = newPipeNum;
        
    }

    public double getPipeline(){
        return pipeNum;
    }

}
