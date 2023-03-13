package frc.robot.utils;

import frc.robot.*;

import java.io.*;
import java.util.*;

import javax.print.attribute.standard.DialogTypeSelection;

public class QuadraticProfile {

    double period;

    public QuadraticProfile(double period) {
        this.period = period;
    }

    public QuadraticProfile() {
        this(.02);
    }

    public R2Vector[] getCombinedSetPoints(R2Vector initialPosition, double[] initialAngles, R2Vector goal, double theta, double speed, double acceleration) {
        R2Vector[] accelSetpoints, stoppingSetpoints, constantSpeedSetpoints;
        
        // displacement from the initial (x, y) to goal 
        R2Vector displacement = goal.sub(initialPosition);
        double angleDisplacement = displacement.dir(); // radians

        double timeToAccelerate = speed / acceleration;

        // check if we'll go past the setpoint/can't achieve max velocity and slow down in time
        if (0.5 * acceleration * Math.pow(timeToAccelerate, 2) > 0.5 * displacement.mag()) {
            // calculate time to go halfway
            timeToAccelerate = Math.sqrt(displacement.mag() / acceleration);

            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement, acceleration * timeToAccelerate, acceleration);
            
            // never reach max velocity so there should be no setpoints at a constant velocity
           R2Vector[] temp = {new R2Vector(0, 0)};
            constantSpeedSetpoints = temp;
        } else {
            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement, acceleration * timeToAccelerate, acceleration);
    
            R2Vector lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
            R2Vector constantSpeedDisplacement = displacement.sub(lastAccelPoint.multiply(2));
            double timeConstantSpeed = constantSpeedDisplacement.mag() / speed;
            constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement, speed);
        }

        // System.out.println("ASUHDUHSUH" + accelSetpoints[45].x);
        System.out.println("#Accel: " + accelSetpoints.length);
        System.out.println("#Speed: " + constantSpeedSetpoints.length);
        System.out.println("#Stop: " + stoppingSetpoints.length);

        return combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);
    }

    public ArrayList<double[]> getSetPoints(R2Vector initialPosition, double[] initialAngles, R2Vector goal, double theta, double speed, double acceleration) {
        double directionX, directionY;
        
        R2Vector[] combined = getCombinedSetPoints(initialPosition, initialAngles, goal, theta, speed, acceleration);
        
        ArrayList<double[]> setpoints = new ArrayList<double[]>(combined.length);
        double[] angles;
        for (int i = 0; i < combined.length; i++) {
            try {
                double x = combined[i].x;
                double y = combined[i].y;
                System.out.println("(x, y) = " + x + ", " + y);
                if (i == 0) {
                    angles = Kinematics.positionInverseKinematics(x, y, true);
                    // angles = Kinematics.positionInverseKinematics(x, y, initialAngles);
                } else {
                    angles = Kinematics.positionInverseKinematics(x, y, true);
                    // angles = Kinematics.positionInverseKinematics(x, y, setpoints.get(i-1));
                }

                double wristPosition = Kinematics.wristDesiredPosition(x, y);
                // if its in between the two thresholds, assume it's flush with upper arm
                if (wristPosition == -1) {
                    wristPosition = 0;
                }
                
                if (i == 0) {
                    directionX = 0;
                    directionY = 0;
                } else {
                    double prevX = combined[i-1].x;
                    double prevY = combined[i-1].y;

                    directionX = x - prevX;
                    directionY = y - prevY;
                }
                    

                if (Double.isNaN(angles[0]) || Double.isNaN(angles[1])) {
                    throw new ArithmeticException("angle is NaN");
                }
                if (Kinematics.isMovingPastLimit(Math.toDegrees(angles[0]), Math.toDegrees(angles[1]), wristPosition, directionX, directionY)) {
                    throw new Exception("(x, y) = (" + x + ", " + y + "), (a, b) = (" + angles[0] + ", " + angles[1] + ") " + "goes past a motor limit");
                }
            } catch(ArithmeticException e) 
            {
                System.out.println("X: " + combined[i].x);
                System.out.println("Y: " + combined[i].y);
                System.out.println(e.getMessage());
                if (i != 0) {
                    setpoints.add(setpoints.get(i-1));
                }
                continue;
            } catch(Exception e) {
                // If the motor is past a limit, stop the sequence
                System.out.println("X: " + combined[i].x);
                System.out.println("Y: " + combined[i].y);
                System.out.println(e.getMessage());
                
                break;
            }
            
            setpoints.add(angles);
        }
        System.out.println("GOT HERE -1");
        return setpoints;
    }

    public R2Vector[] getAcceleratingSetPoints(double timeToEnd, double angle, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        R2Vector[] angles = new R2Vector[numSteps]; 

        for (int i=0; i < numSteps; i++) {
            angles[i] = new R2Vector(
                0.5*Math.cos(angle)*acceleration* Math.pow(i*period, 2),
                0.5*Math.sin(angle)*acceleration* Math.pow(i*period, 2)
            );
        }
        return angles;
    }

    public R2Vector[] getStoppingSetpoints(double timeToEnd, double angle, double velocity, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        R2Vector[] angles = new R2Vector[numSteps];

        for (int i=0; i < numSteps; i++) {
            double dt = i*period;
            angles[i] = new R2Vector(
                (velocity - 0.5*acceleration*dt) * Math.cos(angle)*dt,
                (velocity - 0.5*acceleration*dt) * Math.sin(angle)*dt
            );
        }
        return angles;
    }

    public R2Vector[] getConstantSpeedSetpoints(double timeToEnd, double angle, double velocity) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        R2Vector[] angles = new R2Vector[numSteps]; 

        for (int i=0; i < numSteps; i++) {
            angles[i] = new R2Vector(
                (velocity * i*period) * Math.cos(angle),
                (velocity * i*period) * Math.sin(angle)
            );
        }
        return angles;
    }

    public R2Vector[] combineSetPoints(R2Vector[] accel, R2Vector[] constant, R2Vector[] stopping, R2Vector initialPosition) {
        R2Vector[] combined = new R2Vector[accel.length + constant.length + stopping.length];
        
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