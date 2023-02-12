package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ZED {
    
    private double x, y, z;
    NetworkTable tags = NetworkTableInstance.getDefault().getTable("tags");

    public double getAprilTagX() {
        x = tags.getEntry("x").getDouble(0);
        return x;
    }

    public double getAprilTagY() {
        y = tags.getEntry("y").getDouble(0);
        return y;
    }

    public double getAprilTagZ() {
        z = tags.getEntry("z").getDouble(0);
        return z;
    }
}
