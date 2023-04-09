package frc.robot.utils;

import java.util.*;

public class AngularBangBang {
    double period;

    public AngularBangBang(double period) {
        this.period = period;
    }

    public AngularBangBang() {
        this(.02);
    }

    public ArrayList<double[]> getSetPoints(ArmState initialPosition, ArmState goal, double acceleration, ArrayList<ArmState> guides) {        
        if (guides.size() == 0) return getSetPoints(initialPosition, goal, acceleration);
        
        ArrayList<double[]> setpoints = new ArrayList<double[]> ();
        for (int i=0; i < guides.size(); i++) {
            ArmState guide = guides.get(i);
            if (i == 0) {
                setpoints.addAll(getSetPoints(initialPosition, guide, acceleration));
            } else {
                setpoints.addAll(getSetPoints(guides.get(i-1), guide, acceleration));
            }
        }
        setpoints.addAll(getSetPoints(guides.get(guides.size()-1), goal, acceleration));
        return setpoints;
    }

    /**
     * @param initialState (ArmState) - Initial State of the arm
     * @param goal (ArmState) - Goal State of the arm
     * @param maxAcceleration (double) - Maximum acceleration of the arm in degrees per second squared
     * @return
     */
    public ArrayList<double[]> getSetPoints(ArmState initialState, ArmState goal, double maxAcceleration) {
        ArrayList<double[]> setpoints = calculateSetPoints(initialState, goal, maxAcceleration);
        return setpoints;
        // return filterForExceptions(setpoints);
    }

    public ArrayList<double[]> calculateSetPoints(ArmState initialState, ArmState goal, double maxAcceleration) {
        ArrayList<double[]> setpoints = new ArrayList<double[]>();

        // displacement from the initial (x, y) to goal 
        double alphaDisplacement = goal.getAlpha() - initialState.getAlpha();
        double betaDisplacement = goal.getBeta() - initialState.getBeta();
        double maxDisplacement = Math.max(Math.abs(alphaDisplacement), Math.abs(betaDisplacement));

        // dx = 1/2 * a * dt^2 - calculate time to go halfway at constant accel
        double timeToHalf = Math.sqrt(maxDisplacement / maxAcceleration);

        // 1/2 dx = 1/2 * a * dt^2 - calculate acceleration to go halfway
        double alphaAccel = alphaDisplacement / Math.pow(timeToHalf, 2);
        double betaAccel = betaDisplacement / Math.pow(timeToHalf, 2);
        double maxAlphaVelocity = alphaAccel * (timeToHalf);
        double maxBetaVelocity = betaAccel * (timeToHalf);

        // start at the initial state and accelerate to halfway
        for (double dt = 0; dt < timeToHalf; dt += period) {
            setpoints.add(new double[] {
                initialState.getAlpha() + 0.5 * alphaAccel * dt*dt,
                initialState.getBeta() + 0.5 * betaAccel * dt*dt
            });
        }

        // start at the halfway point and decellerate to end
        for (double dt = 0; dt < timeToHalf; dt += period) {
            setpoints.add(new double[] {
                (initialState.getAlpha() + goal.getAlpha())*0.5 + (maxAlphaVelocity * dt) - (0.5 * alphaAccel * dt*dt),
                (initialState.getBeta() + goal.getBeta())*0.5 + (maxBetaVelocity * dt) - (0.5 * betaAccel * dt*dt)
            });
        }

        return setpoints;
    }

    public ArrayList<double[]> filterForExceptions(ArrayList<double[]> setpoints) {
        double[] angles;
        double directionX, directionY;

        for (int i = 0; i < setpoints.size(); i++) {
            try {
                angles = setpoints.get(i);
                double alpha = angles[0];
                double beta = angles[1];


                double[] pos = Kinematics.forwardKinematics(alpha, beta);
                double x = pos[0]; 
                double y = pos[1];

                double wristPosition = Kinematics.wristDesiredPosition(x, y);
                // if its in between the two thresholds, assume it's flush with upper arm
                if (wristPosition == -1) {
                    wristPosition = 0;
                }
                
                if (i != 0) {
                    double[] prevPos = Kinematics.forwardKinematics(alpha, beta);
                    double prevX = prevPos[0];
                    double prevY = prevPos[1];

                    directionX = x - prevX;
                    directionY = y - prevY;

                    if (Kinematics.isMovingPastLimit(alpha, beta, wristPosition, directionX, directionY)) {
                        // throw new Exception("(x, y) = (" + x + ", " + y + "), (a, b) = (" + angles[0] + ", " + angles[1] + ") " + "goes past a motor limit");
                    }
                }
                    
                if (!Kinematics.isPositionPossible(x, y)) {
                    // throw new ArithmeticException("(x, y) = (" + x + ", " + y + "), (a, b) = (" + angles[0] + ", " + angles[1] + ") " + "is not physically possible");
                }
                if (Double.isNaN(alpha) || Double.isNaN(beta)) {
                    throw new ArithmeticException("SOMETHING MESSED UP - angle is NaN");
                }
            } catch(ArithmeticException e) 
            {
                // System.out.println("X: " + combined[i].x);
                // System.out.println("Y: " + combined[i].y);
                System.out.println(e.getMessage());
                if (i != 0) {
                    setpoints.set(i, setpoints.get(i-1));
                }
                continue;
            } catch(Exception e) {
                // If the motor is past a limit, stop the sequence
                // System.out.println("X: " + combined[i].x);
                // System.out.println("Y: " + combined[i].y);
                ArrayList<double[]> newSetpoints = new ArrayList<double[]>(i);
                for (int j=0; j < i; j++) {
                    newSetpoints.add(setpoints.get(j));
                }
                setpoints = newSetpoints;
                System.out.println(e.getMessage());
                break;
            }
        }
        System.out.println("GOT HERE -1");
        return setpoints;
    }
}
