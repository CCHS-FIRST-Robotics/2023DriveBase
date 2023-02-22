// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.*;

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
    LinearProfile(double period) {
        this.period = period;
    }

    LinearProfile() {
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
     *                                  setpoints[i] gives an array of the angle for each joint
     */
    public double[][] getSetPoints(Vector initialPosition, Vector goal) {
        Vector displacement = goal.sub(initialPosition);
        
        // ∆t = ∆x/∆v
        double timeToEnd = displacement.mag() / Constants.ARM_MAX_SPEED;
        double angle = displacement.dir(); // radians

        /* The number of setpoints we need - one setpoint every timestep (default 20ms)
                                             for however long the movement takes */
        int numberOfSteps = (int) (Math.ceil(timeToEnd / this.period));
        double[][] setpoints = new double[numberOfSteps][2];

        // Calculate the new setpoint for each timestep
        for (int stepsTaken = 0; stepsTaken < numberOfSteps; stepsTaken++) {
            // xf = xi + v∆t in 1D -> convert to 2d based on the angle we're traveling
            Vector pos = new Vector(
                stepsTaken * this.period * Constants.ARM_MAX_SPEED * Math.cos(angle),
                stepsTaken * this.period * Constants.ARM_MAX_SPEED * Math.sin(angle)
            ).add(initialPosition);
            setpoints[stepsTaken] = Kinematics.positionInverseKinematics(pos.x, pos.y);
        }
        
        return setpoints;
    }

    // same as above but uses desired time rather than a max velocity
    public double[][] getSetPoints(Vector initialPosition, Vector goal, double timeToEnd) {
        Vector displacement = goal.sub(initialPosition);
        
        double velocity = displacement.mag() / timeToEnd;
        double angle = displacement.dir(); // radians

        int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
        double[][] setpoints = new double[numberOfSteps][2];

        for (int stepsTaken = 0; stepsTaken < numberOfSteps; stepsTaken++) {
            Vector pos = new Vector(
                stepsTaken * this.period * velocity * Math.cos(angle),
                stepsTaken * this.period * velocity * Math.sin(angle)
            ).add(initialPosition);
            setpoints[stepsTaken] = Kinematics.positionInverseKinematics(pos.x, pos.y);
        }
        
        return setpoints;
    }

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


