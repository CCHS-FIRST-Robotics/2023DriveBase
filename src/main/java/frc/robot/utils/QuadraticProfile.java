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
    
    public double[][] getSetPoints(Vector initialPosition, Vector initialVelocity, Vector goal, double theta, double speed, double acceleration) {
        Vector[] accelSetpoints, stoppingSetpoints, constantSpeedSetpoints;
        
        // displacement from the initial (x, y) to goal 
        Vector displacement = goal.sub(initialPosition);
        double angleDisplacement = displacement.dir(); // radians

        Vector maxVelocity = new Vector(speed * Math.cos(angleDisplacement), speed * Math.sin(angleDisplacement));
        Vector changeInVelocity = maxVelocity.sub(initialVelocity);

        double timeToAccelerate = changeInVelocity.mag() / acceleration;
        double accelerationAngle = changeInVelocity.dir();
        double timeToStop = maxVelocity.mag() / acceleration;

        // check if we'll go past the setpoint/can't achieve max velocity and slow down in time
        if (0.5 * acceleration * Math.pow(timeToAccelerate, 2) > 0.5 * displacement.mag()) {
            // calculate time to go halfway
            timeToAccelerate = Math.sqrt(displacement.mag() / acceleration);

            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement);
            
            // never reach max velocity so there should be no setpoints at a constant velocity
            Vector[] temp = {new Vector(0, 0)};
            constantSpeedSetpoints = temp;
        } else {
            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement);
    
            Vector lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
            Vector constantSpeedDisplacement = displacement.sub(lastAccelPoint.multiply(2));
            double timeConstantSpeed = constantSpeedDisplacement.mag() / Constants.ARM_MAX_SPEED;
            constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement);
        }

        // System.out.println("ASUHDUHSUH" + accelSetpoints[45].x);
        System.out.println("#Accel: " + accelSetpoints.length);
        System.out.println("#Speed: " + constantSpeedSetpoints.length);
        System.out.println("#Stop: " + stoppingSetpoints.length);

        Vector[] combined = combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);

        double[][] setpoints = new double[combined.length][2];
        for (int i = 0; i < combined.length; i++) {
            double[] angles = Kinematics.positionInverseKinematics(combined[i].x, combined[i].y, theta);
            setpoints[i][0] = angles[0];
            setpoints[i][1] = angles[1];
        }

        return setpoints;
    }

    public double[][] getSetPoints(Vector initialPosition, Vector goal, double theta, double speed, double acceleration) {
        Vector[] accelSetpoints, stoppingSetpoints, constantSpeedSetpoints;
        
        // displacement from the initial (x, y) to goal 
        Vector displacement = goal.sub(initialPosition);
        double angleDisplacement = displacement.dir(); // radians

        double timeToAccelerate = speed / acceleration;

        // check if we'll go past the setpoint/can't achieve max velocity and slow down in time
        if (0.5 * acceleration * Math.pow(timeToAccelerate, 2) > 0.5 * displacement.mag()) {
            // calculate time to go halfway
            timeToAccelerate = Math.sqrt(displacement.mag() / acceleration);

            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement);
            
            // never reach max velocity so there should be no setpoints at a constant velocity
            Vector[] temp = {new Vector(0, 0)};
            constantSpeedSetpoints = temp;
        } else {
            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement);
    
            Vector lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
            Vector constantSpeedDisplacement = displacement.sub(lastAccelPoint.multiply(2));
            double timeConstantSpeed = constantSpeedDisplacement.mag() / Constants.ARM_MAX_SPEED;
            constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement);
        }

        // System.out.println("ASUHDUHSUH" + accelSetpoints[45].x);
        System.out.println("#Accel: " + accelSetpoints.length);
        System.out.println("#Speed: " + constantSpeedSetpoints.length);
        System.out.println("#Stop: " + stoppingSetpoints.length);

        Vector[] combined = combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);

        double[][] setpoints = new double[combined.length][2];
        double[] angles;
        for (int i = 0; i < combined.length; i++) {
            try {
                angles = Kinematics.positionInverseKinematics(combined[i].x, combined[i].y, theta);
                if (Double.isNaN(angles[0]) || Double.isNaN(angles[1])) {
                    throw new Exception("angle is NaN");
                }
            } catch(Exception e) 
            {
                System.out.println("X: " + combined[i].x);
                System.out.println("Y: " + combined[i].y);
                System.out.println(e.getMessage());
                if (i != 0) {
                    setpoints[i][0] = setpoints[i-1][0];
                    setpoints[i][1] = setpoints[i-1][1];
                }
                continue;
            }
            
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
                0.5*Math.cos(angle)*Constants.ARM_MAX_ACCELERATION* Math.pow(i*period, 2),
                0.5*Math.sin(angle)*Constants.ARM_MAX_ACCELERATION* Math.pow(i*period, 2)
            );
        }
        return angles;
    }

    public Vector[] getStoppingSetpoints(double timeToEnd, double angle) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Vector[] angles = new Vector[numSteps];

        for (int i=0; i < numSteps; i++) {
            angles[i] = new Vector(
                (Constants.ARM_MAX_SPEED - 0.5*Constants.ARM_MAX_ACCELERATION* i*period) * Math.cos(angle)*i*period,
                (Constants.ARM_MAX_SPEED - 0.5*Constants.ARM_MAX_ACCELERATION* i*period) * Math.sin(angle)*i*period
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
