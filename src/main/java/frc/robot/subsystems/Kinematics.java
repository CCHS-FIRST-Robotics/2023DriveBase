package frc.robot.subsystems;
import frc.robot.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kinematics {
    /**
	 * This will return the desired x, y pos for the given angle of each length
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
     * @param beta (double) - angle of elbow from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the end effector, in meters
	 */
    public static double[] forwardKinematics(double alpha, double beta) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(beta));

		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(beta));

		double[] pos = {x, y + Constants.SHOULDER_JOINT_HEIGHT}; // shoulder joint is ~.59 meters off the ground
		return pos;
    }

	/**
	 * This will return the desired x, y pos for the given angle of each length - includes wrist
	 * ASSUMES WRIST IS A REVOLVING JOINT ON THE SAME PLANE AS THE OTHER LENGTHS
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
     * @param beta (double) - angle of elbow from horizontal, in degrees
	 * @param theta (double) - angle of wrist from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the end effector, in meters
	 */
	public static double[] forwardKinematics(double alpha, double beta, double theta) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(beta)) +
					Constants.WRIST_LENGTH * Math.cos(Math.toRadians(theta));

		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(beta)) +
					Constants.WRIST_LENGTH * Math.sin(Math.toRadians(theta));

		double[] pos = {x, y};
		return pos;
	}


    /**
	 * This will return the desired angle of each arm length using IK
	 * Calculated by finding the (x, y) coords of the center joint by using 
	 * the intersection of the two circles at the joint and end effector
	 * with radius equal to the arm segement length
	 * Intersection solution from: https://math.stackexchange.com/a/1033561
	 * 
	 * @param xPos (double)
     * @param yPos (double)
	 * @return angles (double[2]) degrees
	 */
    public static double[] positionInverseKinematics(double xPos, double yPos) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;

		yPos -= Constants.SHOULDER_JOINT_HEIGHT;

		double dist = Math.sqrt(xPos*xPos + yPos*yPos);
		double l = (l1*l1 - l2*l2 + dist*dist) / (2*dist);
		double h = Math.sqrt(l1*l1 - l*l);

		// double x1 = l/dist * xPos - h/dist * yPos;
    	double y1 = l/dist * yPos + h/dist * xPos;

		double alpha = Math.asin(y1/l1);
    	double beta = -Math.PI/2 + alpha - Math.asin((l1*l1 + l2*l2 - dist*dist)/(2*l1*l2));
		
		SmartDashboard.putNumber("DESIRED ALPHA", Math.toDegrees(alpha));
		SmartDashboard.putNumber("DESIRED BETA", Math.toDegrees(beta));

		double[] angles = {alpha, beta};
        return angles;
    }

	/**
	 * This will return the desired speed of each motor using IK
	 * 
	 * @param xSpeed (double) percent [0, 1]
     * @param ySpeed (double) percent [0, 1]
     * @param alpha (double) angle of shoulder from horizontal, in degrees
     * @param beta (double) angle of elbow from horizontal, in degrees
	 * @return speeds (double[2]) motor speeds - percent [0, 1]
	 */
    public static double[] speedInverseKinematics(double xSpeed, double ySpeed, double alpha, double beta) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;

		// proportional linear speed of the lower arm
		double v1x = l1 * Math.cos(alpha);
		double v1y = l1 * Math.sin(alpha);

		// proportional linear speed of the upper arm
		double v2x = l2 * Math.cos(beta);
		double v2y = l2 * Math.sin(beta);

		// porportional angular speed of each arm linkage
		double w1x, w2x, w1y, w2y;

		// LATERAL MOVEMENT: derivations on readme
		if (v2y > v1y) {
			w1x = 1;
			w2x = -v1y/v2y * w1x;
		} else {
			w2x = 1;
			w1x = -v2y/v1y * w2x;
		}

		// VERTICAL MOVEMENT: derivations on readme
		if (v2x > v1x) {
			w1y = 1;
			w2y = -v1x/v2x * w1y;
		} else {
			w2y = 1;
			w1y = -v2x/v1x * w2y;
		}

		// Scale by controller input:
		w1x *= xSpeed;
		w2x *= xSpeed;

		w1y *= ySpeed;
		w2y *= ySpeed;

		double[] combinedSpeeds = {w1x + w1y, w2x + w2y};

        return combinedSpeeds;
    }
}
