package frc.robot.utils;

import frc.robot.Constants;

public class ArmState {
    double l1 = Constants.LOWER_ARM_LENGTH;
    double l2 = Constants.UPPER_ARM_LENGTH;

    double x, y, alpha, beta;
    R2Vector position;
    boolean inverted;

    public ArmState() {
        this.x = 0;
        this.y = 0;
        this.position = new R2Vector();
        
        this.alpha = Constants.SHOULDER_STARTING_ANGLE;
        this.beta = Constants.ELBOW_STARTING_ANGLE;
        
        this.inverted = false;
    }

    public ArmState(double x, double y, boolean inverted) {
        this.x = x;
        this.y = y;
        this.position = new R2Vector(x, y);

        double[] angles = Kinematics.positionInverseKinematics(x, y, inverted);
        this.alpha = angles[0];
        this.beta = angles[1];

        this.inverted = inverted;
    }

    public ArmState(double alpha, double beta) {
        this.alpha = alpha;
        this.beta = beta;

        double[] pos = Kinematics.forwardKinematics(alpha, beta);
        this.x = pos[0];
        this.y = pos[1];

        this.inverted = (Kinematics.positionInverseKinematics(x, y, true)[0] == alpha);
    }

    public double[] degrees() {
        return new double[] {alpha, beta};
    }

    public double[] radians() {
        return new double[] {Math.toRadians(alpha), Math.toRadians(beta)};
    }

    public boolean isInverted() {
        return inverted;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public R2Vector getPosition() {
        return position;
    }
}
