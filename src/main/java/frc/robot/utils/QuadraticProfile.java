package frc.robot.utils;
import frc.robot.*;

public class TrapezoidProfile {

    double period;

    TrapezoidProfile(double period) {
        this.period = period;
    }

    TrapezoidProfile() {
        this(.02);
    }
    
    public double[][] getSetPoints(Vector initialPosition, Vector goal, Vector initialVelocity) {
        // The max linea speed and acceleration we want the arm to travel at
        double speed = Constants.ARM_MAX_SPEED;
        double acceleration = Constants.ARM_MAX_ACCELERATION;

        // displacement from the initial (x, y) to goal 
        Vector displacement = goal.sub(initialPosition);
        double angleDisplacement = displacement.dir(); // radians

        // velocity vector at the max speed in the desired direction
        Vector maxVelocity = new Vector(
            speed*Math.cos(angleDisplacement), 
            speed*Math.sin(angleDisplacement)
        );

        Vector changeInVelocity = maxVelocity.sub(initialVelocity);
        double angleAcceleration = changeInVelocity.dir();

        double timeToAccelerate = changeInVelocity.mag() / acceleration;
        double distanceAccelerating = (initialVelocity.mag() + 1/2 * acceleration * timeToAccelerate) * timeToAccelerate;
        double distanceStopping = (speed )

        double timeToStop = maxVelocity.mag() / Constants.ARM_MAX_ACCELERATION;
        
        double timeConstant = (displacement.mag() -  ) / Constants.ARM_MAX_SPEED;
        

        int numberOfSteps = (int) (Math.ceil(timeToEnd / Constants.PERIOD));
        double[][] setpoints = new double[numberOfSteps][2];

        for (int i = 0; i < numberOfSteps; i++) {
            Vector pos = new Vector(
                i * Constants.PERIOD * Constants.ARM_MAX_SPEED * Math.cos(angle),
                i * Constants.PERIOD * Constants.ARM_MAX_SPEED * Math.sin(angle)
            ).add(initialPosition);
            setpoints[i] = Kinematics.positionInverseKinematics(pos.x, pos.y);
        }
        
        return setpoints;
    }

    public double[][] getConstantSetpoints(Vector initialPosition, double timeToEnd, double angle) {

    }
}
