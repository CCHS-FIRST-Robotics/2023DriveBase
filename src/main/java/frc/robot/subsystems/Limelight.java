package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    // What each of the variables from the pipline represent:
    //   tv --> Whether the limelight has any valid targets (0 or 1)
    //   tx --> Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    //   ty --> Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    //   ta --> Target Area (0% of image to 100% of image)
    // For more information refer to: https://docs.limelightvision.io/en/latest/getting_started.html 
    private double x, y, area, forwardDistance;
    private int pipeNum;

    // This makes the network table so that the liemlight can pass down new values via the pipeline
    // There are multiple pipelines so that different sets of values can be passed down
    // Each pipline is tuned for its respective target, and its values give information on that target
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // This calculates the forward distance using trig
    // The forward distance is the equivalent of adjacent side in the triangle
    // The highet from the lens to the target is the opposite side of the triangle
    // With the measured verticle angle, we can use tangent to calculate the adjacent side (aka the forward distance)
    // For more help go to: https://www.dummies.com/article/academics-the-arts/math/trigonometry/trigonometry-for-dummies-cheat-sheet-207754/ 
    public double getForwardDistance(int pipeChoice) {
        changePipeline(pipeChoice);

        double goalHeight = 0;

        // This toggles between target heights (needed for calculating the opposite side)
        if (pipeChoice == 0) {
            goalHeight = Constants.SHORT_TARGET_HEIGHT;
        } 
        else if (pipeChoice == 1) {
            goalHeight = Constants.TALL_TARGET_HEIGHT;
        }

        double limelightLensHeight        = Constants.LIME_HEIGHT;
        double limelightMountAngle        = Constants.LIME_ANGLE;
        double targetOffsetAngleVertical  = table.getEntry("ty").getDouble(0.0) * ((Math.PI)/(180));
        // Last part of that line converts the angle to radians

        double angleToGoal = limelightMountAngle + targetOffsetAngleVertical;

        forwardDistance = (goalHeight - limelightLensHeight)/Math.tan(angleToGoal);

        return forwardDistance;
    }
    
    // Not sure why this is here --> we need to take this out later
    public double getHeadingDisplacement(int pipeChoice) {
        changePipeline(pipeChoice);
        return table.getEntry("tx").getDouble(0.0) * ((Math.PI)/(180));
    }

    // This passes back how many degrees it is off by horixontally
    public double getX(int pipeChoice) {
        changePipeline(pipeChoice);
        x = table.getEntry("tx").getDouble(0);
        return x;
    }
    
    // This passes back how many degrees it is off by vertically
    public double getY(int pipeChoice) {
        changePipeline(pipeChoice);
        y = table.getEntry("ty").getDouble(0);
        return y;
   }
    
   // This passes back how much of the camera lens the target fills
    public double getArea(int pipeChoice) {
        changePipeline(pipeChoice);
        area = table.getEntry("ta").getDouble(0);
        return area;
    }

    // This should (in theory) switch the pipeline so that we can look at one target then another
    // In the 2023 game this means that we can switch between the high and low reflective tape on the poles
    // Pipeline 0 is for the low target
    // Pipeline 1 is for the high target
    public void changePipeline(int newPipeNum){
        if (!table.getEntry("pipeline").setInteger(newPipeNum))
        {
            System.out.println("Failed to set pipline");
        }
        else{
            pipeNum = newPipeNum;
            
            System.out.println(pipeNum);
            System.out.println(newPipeNum);

        }

        System.out.println(getPipeline());
        //table.getEntry("pipeline").setInteger(newPipeNum);

        pipeNum = newPipeNum;
    }

    // Returns the last number the pipeline was supposed to be
    // Currently, this may not be accurate because the pipeline seems to have issues switching
    public int getPipeline(){
        return pipeNum;
    }

}
