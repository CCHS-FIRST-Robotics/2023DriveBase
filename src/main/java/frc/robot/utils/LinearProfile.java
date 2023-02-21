// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.*;

/** Add your docs here. */
public class LinearProfile {

    double period;

    LinearProfile(double period) {
        this.period = period;
    }

    LinearProfile() {
        this(.02);
    }
    
    public double[][] getSetPoints(Vector initialPosition, Vector goal) {
        Vector displacement = goal.sub(initialPosition);
        
        double timeToEnd = displacement.mag() / Constants.ARM_MAX_VELOCITY;
        double angle = displacement.dir(); // radians

        int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
        double[][] setpoints = new double[numberOfSteps][2];

        for (int i = 0; i < numberOfSteps; i++) {
            Vector pos = new Vector(
                i * Constants.PERIOD * Constants.ARM_MAX_VELOCITY * Math.cos(angle),
                i * Constants.PERIOD * Constants.ARM_MAX_VELOCITY * Math.sin(angle)
            ).add(initialPosition);
            setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y);
        }
        
        return setpoints;
    }

    // uses desired time rather than a max velocity
    public double[][] getSetPoints(Vector initialPosition, Vector goal, double timeToEnd) {
        Vector displacement = goal.sub(initialPosition);
        
        double velocity = displacement.mag() / timeToEnd;
        double angle = displacement.dir(); // radians

        int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
        double[][] setpoints = new double[numberOfSteps][2];

        for (int i = 0; i < numberOfSteps; i++) {
            Vector pos = new Vector(
                i * Constants.PERIOD * velocity * Math.cos(angle),
                i * Constants.PERIOD * velocity * Math.sin(angle)
            ).add(initialPosition);
            setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y);
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


