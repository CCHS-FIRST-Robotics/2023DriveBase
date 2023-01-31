package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    double x;
    double y;
    double area;

    NetworkTableEntry tx, ty, ta;

    public void limelight() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        x    = tx.getDouble(0.0);
        y    = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

    }

    public void printVal() {
        System.out.println("limeval " + x);
    }

    public void smartDash() {

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }
}
