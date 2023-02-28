package frc.robot.utils;

import frc.robot.*;

public class QuadraticProfile {

    double period;

    public QuadraticProfile(double period) {
        this.period = period;
    }

    public QuadraticProfile() {
        this(.02);
    }
    
    // public double[][] getSetPoints(Vector initialPosition, Vector goal, Vector initialVelocity) {
    //     // The max linea speed and acceleration we want the arm to travel at
    //     double speed = Constants.ARM_MAX_SPEED;
    //     double acceleration = Constants.ARM_MAX_ACCELERATION;

    //     // displacement from the initial (x, y) to goal 
    //     Vector displacement = goal.sub(initialPosition);
    //     double angleDisplacement = displacement.dir(); // radians

    //     // velocity vector at the max speed in the desired direction
    //     Vector maxVelocity = new Vector(
    //         speed*Math.cos(angleDisplacement), 
    //         speed*Math.sin(angleDisplacement)
    //     );

    //     Vector changeInVelocity = maxVelocity.sub(initialVelocity);
    //     double angleAcceleration = changeInVelocity.dir();

    //     double timeToAccelerate = changeInVelocity.mag() / acceleration;
    //     double distanceAccelerating = (initialVelocity.mag() + 1/2 * acceleration * timeToAccelerate) * timeToAccelerate;
    //     double distanceStopping = (speed )

    //     double timeToStop = maxVelocity.mag() / Constants.ARM_MAX_ACCELERATION;
        
    //     double timeConstant = (displacement.mag() -  ) / Constants.ARM_MAX_SPEED;
        

    //     int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
    //     double[][] setpoints = new double[numberOfSteps][2];

    //     for (int i = 0; i < numberOfSteps; i++) {
    //         Vector pos = new Vector(
    //             i * Constants.PERIOD * Constants.ARM_MAX_SPEED * Math.cos(angle),
    //             i * Constants.PERIOD * Constants.ARM_MAX_SPEED * Math.sin(angle)
    //         ).add(initialPosition);
    //         setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y);
    //     }
        
    //     return setpoints;
    // }

    public double[][] getSetPoints(Vector initialPosition, Vector goal, double theta) {
        // The max linea speed and acceleration we want the arm to travel at
        double speed = Constants.ARM_MAX_SPEED;
        double acceleration = Constants.ARM_MAX_ACCELERATION;

        // displacement from the initial (x, y) to goal 
        Vector displacement = goal.sub(initialPosition);
        double angleDisplacement = displacement.dir(); // radians

        double timeToAccelerate = speed / acceleration;
        Vector[] accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement);
        Vector[] stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement);

        Vector lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
        Vector constantSpeedDisplacement = displacement.sub(lastAccelPoint.multiply(2));
        double timeConstantSpeed = constantSpeedDisplacement.mag() / Constants.ARM_MAX_SPEED;
        Vector[] constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement);

        System.out.println("ASUHDUHSUH" + accelSetpoints[45].x);

        Vector[] combined = combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);

        double[][] setpoints = new double[combined.length][2];
        for (int i = 0; i < combined.length; i++) {
            double[] angles = Kinematics.positionInverseKinematics(combined[i].x, combined[i].y, theta);
            setpoints[i][0] = angles[0];
            setpoints[i][1] = angles[1];
        }

        return setpoints;
    }

    public Vector[] getAcceleratingSetPoints(double timeToEnd, double angle) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Vector[] angles = new Vector[numSteps]; 

        for (int i=0; i < numSteps; i++) {
            angles[i] = new Vector(
                1/2*Math.cos(angle)*Constants.ARM_MAX_ACCELERATION* Math.pow(i*period, 2),
                1/2*Math.sin(angle)*Constants.ARM_MAX_ACCELERATION* Math.pow(i*period, 2)
            );
        }
        return angles;
    }

    public Vector[] getStoppingSetpoints(double timeToEnd, double angle) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Vector[] angles = new Vector[numSteps];

        for (int i=0; i < numSteps; i++) {
            angles[i] = new Vector(
                (Constants.ARM_MAX_SPEED - 1/2*Constants.ARM_MAX_ACCELERATION* i*period) * Math.cos(angle)*i*period,
                (Constants.ARM_MAX_SPEED - 1/2*Constants.ARM_MAX_ACCELERATION* i*period) * Math.sin(angle)*i*period
            );
        }
        return angles;
    }

    public Vector[] getConstantSpeedSetpoints(double timeToEnd, double angle) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Vector[] angles = new Vector[numSteps]; 

        for (int i=0; i < numSteps; i++) {
            angles[i] = new Vector(
                (Constants.ARM_MAX_SPEED * i*period) * Math.cos(angle),
                (Constants.ARM_MAX_SPEED * i*period) * Math.sin(angle)
            );
        }
        return angles;
    }

    public Vector[] combineSetPoints(Vector[] accel, Vector[] constant, Vector[] stopping, Vector initialPosition) {
        Vector[] combined = new Vector[accel.length + constant.length + stopping.length];
        
        for(int i = 0; i < accel.length; i++) {
            combined[i] = accel[i].add(initialPosition);
        }
        for(int i = 0; i < constant.length; i++) {
            combined[i + accel.length] = constant[i].add(combined[accel.length - 1]);
        }
        for(int i = 0; i < stopping.length; i++) {
            combined[i + accel.length + constant.length] = stopping[i].add(combined[accel.length + constant.length - 1]); 
        }

        return combined;
    }
}
