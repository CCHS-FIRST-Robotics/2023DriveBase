package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ZED {
    
    private double x, y, z, dist;
    private int id;
    NetworkTable tags = NetworkTableInstance.getDefault().getTable("tags");

    public enum Position {
        CUBE, CONE, SUBSTATION
    }

    public double[] getAprilTagPos(Position desiredPos) {
        double[] pos = getAprilTagPos();
        switch (desiredPos) {
            case CUBE:
                pos[0] = pos[0] - Constants.ZED_OFFSET;
                return pos;
            case CONE:
                pos[0] = pos[0] - (Constants.ZED_OFFSET + Constants.CONE_FROM_TAG_OFFSET);
                return pos;
            case SUBSTATION:
                return getAprilTagPos(Constants.SUBSTATION_TAG_ID);
            default:
                return new double[] {0, 0, 0};
        }
    }

    public int getAprilTagId() {
        id = (int) tags.getEntry("id").getDouble(0);
        return id;
    }

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

    public double[] getAprilTagPos() {
        return new double[] {getAprilTagX(), getAprilTagY(), getAprilTagZ()};
    }

    public double[] getAprilTagPos(String id) {
        return tags.getEntry(id).getDoubleArray(new double[] {0, 0, 0});
    }

    public double getAprilTagDist() {
        dist = tags.getEntry("dist").getDouble(0);
        return dist;
    }
}
