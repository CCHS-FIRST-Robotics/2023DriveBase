package frc.robot.subsystems;

import frc.robot.Constants;
import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterShuffleboard {

    ShuffleboardTab limelightTab; // used for limelight data
    GenericEntry highPostX, lowPostX, highPostY, lowPostY, highPostHeading, lowPostHeading, 
                 highForwardDistance, lowForwardDistance;

    public BetterShuffleboard() {

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
    }

    public void pushDashboard(Limelight limelight){
        pushLimelight(limelight);
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

}
