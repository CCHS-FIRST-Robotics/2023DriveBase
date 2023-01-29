package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Random;

public class Network {
    NetworkTableInstance instance;

    // Positional
    NetworkTableEntry posX;
    NetworkTableEntry posY;
    NetworkTableEntry heading;

    // DEBUG
    NetworkTableEntry currentFPS;
    NetworkTableEntry droppedFrames;


    public Network() {
        this.instance = NetworkTableInstance.getDefault();
    }
    

    public void receiveTable() {
        NetworkTable state = instance.getTable("state");

        posX = state.getEntry("x_pos");
        posX.setDouble(0);

        posY = state.getEntry("y_pos");
        posY.setDouble(0);

        heading = state.getEntry("heading");
        heading.setDouble(0);
    }

    public void receiveDebug() {
        NetworkTable debug = instance.getTable("debug");

        currentFPS = debug.getEntry("fps");
        currentFPS.setDouble(0);

        droppedFrames = debug.getEntry("dropped");
        droppedFrames.setDouble(0);
    }

    public void sendTable() {

    }
}
