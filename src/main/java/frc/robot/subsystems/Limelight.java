package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private double x, y, area;
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


    // TODO: include comments/documentation for what these vals actually are 
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public void printVal() {
        System.out.println("limeval " + x);
    }

    public void test(){
        SmartDashboard.putNumber("test2", test2);
        test2 += 1;
    }
}
