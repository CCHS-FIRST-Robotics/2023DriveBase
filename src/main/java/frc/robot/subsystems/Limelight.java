package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private double x, y, area;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
