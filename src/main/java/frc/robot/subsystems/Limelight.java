package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    double x, y, area;
    NetworkTableEntry tx, ty, ta;
    NetworkTable table;
    double test2 = 0;

    public void limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        x    = tx.getDouble(0);
        y    = ty.getDouble(0);
        area = ta.getDouble(0);

    }

    public void printVal() {
        System.out.println("limeval " + x);
    }

    public void smartDash() {

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    public void test(){
        SmartDashboard.putNumber("test2", test2);
        test2 += 1;
    }
}
