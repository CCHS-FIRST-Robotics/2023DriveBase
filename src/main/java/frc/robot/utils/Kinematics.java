package frc.robot.utils;
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
    public static double[] positionInverseKinematicsOld(double xPos, double yPos) {
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
	 * This will return the desired angle of each arm length using IK
	 * Calculated by finding the (x, y) coords of the center joint
	 * 
	 * Previous method had limitations in that I didn't fully implement the second angle
	 * and the use of asin restricted the possible solutions.
	 * 
	 * Solution derived using MATLAB, I'm using short/unclear variable names 
	 * because they are MASSIVE equations.
	 * 
	 * Refer to https://www.mathworks.com/help/symbolic/derive-and-apply-inverse-kinematics-to-robot-arm.html
	 * for more details.
	 * 
	 * @param x
	 * @param y
	 * @param clawDown (boolean) - whether or not to choose the angle pair that reults in the claw pointing down
	 * 								if the desired position is not possible, it will be ignored
	 * @return angles (double[2]) - angle measurements of each joint in degrees, shoulder first
	 */
	public static double[] positionInverseKinematics(double x, double y, boolean clawDown) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;
		y -= Constants.SHOULDER_JOINT_HEIGHT; // calculations assume the bottom joint is at (0, 0)

		double sigma1 = Math.sqrt(
			-pow(l1,4) 
			+ 2*pow(l1,2)*pow(l2,2) 
			+ 2*pow(l1,2)*pow(x,2)
			+ 2*pow(l1,2)*pow(y,2)
			- pow(l2,4)
			+ 2*pow(l2,2)*pow(x,2)
			+ 2*pow(l2,2)*pow(y,2)
			- pow(x,4)
			- 2*pow(x,2)*pow(y,2)
			- pow(y,4)
		);

		double[] alphas = {
        	2*Math.atan( (2*l1*y+sigma1) / (pow(l1,2) + 2*l1*x - pow(l2,2) + pow(x,2) + pow(y,2)) ),
        	2*Math.atan( (2*l1*y-sigma1) / (pow(l1,2) + 2*l1*x - pow(l2,2) + pow(x,2) + pow(y,2)) )
		};

		double sigma2 = Math.sqrt( 
			(pow(l1,2) + 2*l1*l2 + pow(l2,2) - pow(x,2) - pow(y,2))
		);

		//TODO: make sure I'm not supposed to add on the alphas later/does doing it now fuck with it
		//		(verify using the python sim - plot_workspace())
		double[] betas = {
			-2*Math.atan(sigma2) + alphas[0],
			2*Math.atan(sigma2) + alphas[1]
		};

		// get rid of the two junk angle pairs
		double[][] pairs = new double[2][2];
		int i, j;
		int k = 0;
		double[] pos = {x, y};

		// check which angle pairs actually come out to the right (x, y) coords
		for (i=0; i<2; i++) {
			for (j=0; j<2; j++) {
				 if (forwardKinematics(alphas[i], betas[j]) == pos) {
					double[] pair = {alphas[i], betas[j]};
					pairs[k] = pair;
					k++;
				 }
			}
		}

		// check if either pair violates a motor limit
		for (i=0; i<2; i++) {
			double alpha = pairs[i][0];
			double beta = pairs[i][1];

			if (shouldMotorStop(alpha, beta)) {
				return pairs[1-i];
			}
		}

		// if both pairs are left, prioritize based on (clawDown) param
		
	}

	public static double[] positionInverseKinematics(double x, double y) {
		// If none is specified, choose the position that typically works
		return positionInverseKinematics(x, y, true);
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

	/**
	 * @return shouldMotorStop (boolean) returns true if the motor is past any of the limits - ignores the limits if motorLimits is false
	 */
	public static boolean shouldMotorStop(double alpha, double beta) {
		double[] pos = Kinematics.forwardKinematics(alpha, beta);
		double x = pos[0];
		double y = pos[1];

		return ((
			// check if the shoulder is too far forward/backward
			alpha < Constants.minAlpha ||
			alpha > Constants.maxAlpha ||

			// check if the elbow is too far forward/backward
			beta < Constants.minBeta ||
			beta > Constants.maxBeta ||

			// check if the arm is fully extended -- dont want it to lock/lose a DOF
			x < Constants.minX ||
			x > Constants.maxX ||

			// check if the arm is too close to the ground or above the height limit
			y < Constants.minY ||
			y > Constants.maxY ||

			// check to make sure the arm isn't hitting the frame or the electrical board
			(Constants.isInFrameX(x) && Constants.isBelowFrame(y)) ||
			(Constants.isInFrameX(x) && Constants.isBelowElectricalBoard(y) && x < 0)
		) && motorLimits) || manualMotorStop; // check if the motor limits are activated or if driver is trying to stop them manually
	}

	/**
	 * Literally just shorthand for Math.pow since I couldn't figure out how to import it directly lol
	 * 
	 * @param x (double) - base
	 * @param degree (double) - exponent
	 * @return x^(degree)
	 */
	public static double pow(double x, double degree) {
		return Math.pow(x, degree);
	}
}
