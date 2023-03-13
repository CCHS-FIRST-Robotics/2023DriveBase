package frc.robot.utils;

import java.util.ArrayList;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.*;

import java.io.*;
import java.util.*;

/**
 * Generates a motion profile/trajectory that constrains speed
 * Assumes infinite acceleration and a constant velocity over the trajectory
 */
public class LinearProfile {

    double period;

    /**
     * Constructor for LinearProfile class
     * 
     * @param period (double) - the time between each setpoint/controller call 
     *                          (default 20ms for the periodic loops of the RIO)
     */
    public LinearProfile(double period) {
        this.period = period;
    }

    public LinearProfile() {
        this(Constants.PERIOD);
    }

    //TODO: instead of using the constants max speed take a parameter to make code more flexible?
    
    /**
     * Given an initial and goal position, calculates a trajectory of 
     * controller setpoints, with a constant velocity - assumes starting from rest
     * 
     * Calculations treat the problem as 1D for simplicity since we are only moving in one direction,
     * but adjust the final results to properly map into the 2D cspace
     * 
     * @param initialPosition (Vector) - the initial position of the end effector
     * @param goal (Vector) - the final (desired) position of the end effector
     * 
     * @return setpoints (double[][]) - 2D array of angular setpoints. 
     *                                  setpoints[i] gives an array of the angle for each joint - RADIANS
     */
    public ArrayList<double[]> getSetPoints(R2Vector initialPosition, double[] initialAngles, R2Vector goal, double theta, double speed) {
        R2Vector deltaPos;
        R2Vector displacement = goal.sub(initialPosition);
        
        // ∆t = ∆x/∆v
        double timeToEnd = displacement.mag() / speed;

        /* The number of setpoints we need - one setpoint every timestep (default 20ms)
                                             for however long the movement takes */
        int numberOfSteps = (int) (Math.ceil(timeToEnd / this.period));
        ArrayList<double[]> setpoints = new ArrayList<double[]>(numberOfSteps);

        // Calculate the new setpoint for each timestep
        for (int i = 0; i < numberOfSteps; i++) {
            double proportion = (double) i / (double) numberOfSteps;
            R2Vector pos = new R2Vector(
                proportion * displacement.x,
                proportion * displacement.y
            ).add(initialPosition);

            System.out.println("POS: " + pos.x + " next " + pos.y);
            // if (!Kinematics.isPositionPossible(i, proportion)) {
            //     System.out.println("")
            //     System.out.println("not possible");
            //     break;
            // }
            // double[] temp = {pos.x, pos.y};
            // setpoints.add(temp);
            
            double wristPosition = Kinematics.wristDesiredPosition(pos.x, pos.y);
                // if its in between the two thresholds, assume it's flush with upper arm
            if (wristPosition == -1) {
                wristPosition = 0;
            }

            double angles[] = Kinematics.positionInverseKinematics(pos.x, pos.y, true);
            
            if (i != 0) {
                proportion = (double) (i-1) / (double) numberOfSteps;
                R2Vector prevPos = new R2Vector(
                    proportion * displacement.x,
                    proportion * displacement.y
                ).add(initialPosition);

                deltaPos = pos.sub(prevPos); 
                System.out.println("dletapos: " + deltaPos.x + " next " + deltaPos.y);
            
                if (Kinematics.isMovingPastLimit(Math.toDegrees(angles[0]), Math.toDegrees(angles[1]), wristPosition, deltaPos.x, deltaPos.y)) {
                    System.out.println("(x, y) = (" + pos.x + ", " + pos.y + "), (a, b) = (" + angles[0] + ", " + angles[1] + ") " + "goes past a motor limit");
                    break;
                }
            }

            if (Double.isNaN(angles[0]) || Double.isNaN(angles[1])) {
                System.out.println("angle is nan");
                break;
            }
            setpoints.add(angles);
        }
        
        return setpoints;
    }

    // same as above but uses desired time rather than a max velocity
    // public ArrayList<double[]> getSetPoints(R2Vector initialPosition, double[] initialAngles, R2Vector goal, double theta, double speed) {
    //     R2Vector displacement = goal.sub(initialPosition);
        
    //     double velocity = displacement.mag() / timeToEnd;
    //     double angle = displacement.dir(); // radians

    //     int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
    //     double[][] setpoints = new double[numberOfSteps][2];

    //     for (int i = 0; i < numberOfSteps; i++) {
    //         R2Vector pos = new R2Vector(
    //             i * this.period * velocity * Math.cos(angle),
    //             i * this.period * velocity * Math.sin(angle)
    //         ).add(initialPosition);

    //         if (i == 0) {
    //             setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y, initialAngles);
    //         } else {
    //             setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y, setpoints[i-1]);
    //         }
    //     }
        
    //     return setpoints;
    // }

    // private static double[] getAcceleratingSetPoints(double timeInterval) {

    //     int numberOfSteps = (int) (Math.ceil(timeInterval / Constants.PERIOD));

    //     double[] setPoints = new double[numberOfSteps];

    //     // loops through all the time increments
    //     for(int i = 0; i < numberOfSteps; i++) {
    //         double positionMag = Constants.ARM_MAX_ACCELERATION / 2 * Constants.PERIOD * Constants.PERIOD * (i+1) * (i+1);
    //     }

    //     return new double[1];
    // }
}


