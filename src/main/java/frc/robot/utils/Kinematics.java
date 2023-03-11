package frc.robot.utils;
import frc.robot.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.*;
import java.util.*;

public final class Kinematics {

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
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(alpha + beta)) +
					Constants.WRIST_LENGTH * Math.cos(Math.toRadians(theta));

		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(alpha + beta)) +
					Constants.WRIST_LENGTH * Math.sin(Math.toRadians(theta));

		double[] pos = {x, y + Constants.SHOULDER_JOINT_HEIGHT};
		return pos;
	}

	/**
	 * This will return the desired x, y pos for the given angle of each length 
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
     * @param beta (double) - angle of elbow from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the wrist joint, in meters
	 */
	public static double[] forwardKinematics(double alpha, double beta) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.cos(Math.toRadians(alpha + beta));
		
		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha)) +
					Constants.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(alpha + beta));

		double[] pos = {x, y + Constants.SHOULDER_JOINT_HEIGHT};
		return pos;
	}

	/**
	 * This will return the desired x, y pos for the given angle of each length 
	 * 
	 * @param alpha (double) - angle of shoulder from horizontal, in degrees
	 * @return position (double[]) - (x, y) position of the elbow joint, in meters
	 */
	public static double[] forwardKinematics(double alpha) {
		double x = 	Constants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(alpha));
		double y = 	Constants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(alpha));

		double[] pos = {x, y + Constants.SHOULDER_JOINT_HEIGHT};
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
		
		SmartDashboard.putNumber("DESIRED ALPHA", degrees(alpha));
		SmartDashboard.putNumber("DESIRED BETA", degrees(beta));

		double[] angles = {alpha, beta};
        return angles;
    }

	/**
	 * This will return the desired angle of each arm length using IK
	 * Calculated by finding the (x, y) coords of the center joint
	 * 
	 * Previous method had limitations in that I didn't fully implement the second solution
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
	 * 								if the desired position is not possible, it will be ignored
	 * @return angles (double[2]) - angle measurements of each joint in degrees, shoulder first
	 */
	public static double[][] positionInverseKinematics(double x, double y) {
		double l1 = Constants.LOWER_ARM_LENGTH;
		double l2 = Constants.UPPER_ARM_LENGTH;
		y -= Constants.SHOULDER_JOINT_HEIGHT; // calculations assume the bottom joint is at (0, 0)

		// TODO: adjust for the wrist:
		

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
			(-pow(l1, 2) + 2*l1*l2 - pow(l2, 2) + pow(x, 2) + pow(y, 2)) *
			(pow(l1,2) + 2*l1*l2 + pow(l2,2) - pow(x,2) - pow(y,2))
		) / (-pow(l1, 2) + 2*l1*l2 - pow(l2, 2) + pow(x, 2) + pow(y, 2));

		//TODO: make sure I'm not supposed to add on the alphas later/does doing it now fuck with it
		//		(verify using the python sim - plot_workspace())
		double[] betas = {
			(-2*Math.atan(sigma2)),
			(2*Math.atan(sigma2))
		};

		return new double[][] {{alphas[0], betas[0]}, {alphas[1], betas[1]}};
	}

	/**
	 * @param x
	 * @param y
	 * @param clawDown (boolean) - whether or not to choose the angle pair that reults in the claw pointing down
	 * @return
	 */
	public static double[] positionInverseKinematics(double x, double y, boolean clawDown) {
		// If none is specified, choose the position that typically works
		double[][] angles = positionInverseKinematics(x, y);
		double[] alphas = new double[] {angles[0][0], angles[1][0]};
		double[] betas = new double[] {angles[0][1], angles[1][1]};
		return evaluateAngles(alphas, betas, clawDown);
	}

	public static double[] positionInverseKinematics(double x, double y, double[] initialAngles) {
		double[][] angles = positionInverseKinematics(x, y);
		return evaluateAngles(angles, initialAngles);
	}

	public static double[] evaluateAngles(double[] alphas, double[] betas, boolean clawDown) {
		int i; // TODO:
		// check if either pair violates a motor limit
		// for (i=0; i<2; i++) {
		// 	double alpha = degrees(alphas[i]);
		// 	double beta = degrees(betas[i]);

		// 	if (shouldMotorStop(alpha, beta, theta)) {
		// 		double[] pair = {alphas[1-i], betas[1-i]};
		// 		return pair;
		// 	}
		// }
		// System.out.println("Angles1: " + degrees(alphas[0]) + "\n" + degrees(betas[0]));
		// System.out.println("Angles2: " + degrees(alphas[1]) + "\n" + degrees(betas[1]));

		// if both pairs are left, prioritize based on (clawDown) param
		if ((Math.sin(alphas[0]) < Math.sin(alphas[1])) == (clawDown)) {
			double[] pair = {alphas[1], betas[1]};
			// System.out.println(0);
			return pair;
		}
		double[] pair = {alphas[0], betas[0]};
		return pair;
	}

	public static double[] evaluateAngles(double[][] angles, double[] initialAngles) {
		double[] closestAngles = new double[2];
		double leastError = Math.pow(10, 10);

		for (int i=0; i<angles.length; i++) {
			double error = angularError(angles[i], initialAngles);

			if (error < leastError) {
				closestAngles = angles[i];
				leastError = error;
			}
		}
		return closestAngles;
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
	 * args in deg
	 * @return shouldMotorStop (boolean) returns true if the motor is past any of the limits
	 */
	public static boolean shouldMotorStop(double alpha, double beta, double theta) {
		double[] pos = Kinematics.forwardKinematics(alpha, beta, theta);
		double x = pos[0];
		double y = pos[1];

		return (
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
		);
	}

	public static boolean isMovingPastLimit(double alpha, double beta, double theta, double prevPosition, double currentPosition) {
		if (!shouldMotorStop(alpha, beta, theta)) {
			return false;
		}

		double x = pos[0];
		double y = pos[1];


		// check if the arm is fully extended -- dont want it to lock/lose a DOF
		if (x < Constants.minX && ) {

		}
		if (x > Constants.maxX) {

		}

		// check if the arm is too close to the ground or above the height limit
		if (y < Constants.minY) {

		}
		if (y > Constants.maxY) {

		}

		// check to make sure the arm isn't hitting the frame
		if (Constants.isInFrameX(x) && Constants.isBelowFrame(y)) {

		}
	}

	public static double wristDesiredPosition(double x, double y) {
		if ((y < Constants.WRIST_MIN_ACTUATE) && !Constants.isInFrameX(x)) {
			return 90;
		} if ((y > Constants.WRIST_MAX_ACTUATE) && !Constants.isInFrameX(x)) {
			return 0;
		}

		return -1;
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

	/**
	 * Literally just shorthand for Math.toDegrees since I couldn't figure out how to import it directly lol
	 * 
	 * @param angle (double) - angle is radians
	 * @return angle (double) - angle in degrees
	 */
	public static double degrees(double angle) {
		return Math.toDegrees(angle);
	}

	/**
	 * Applies degrees() to an array - why can't I just have numpy ;(
	 * 
	 * @param angles (double[]) - angles in radians
	 * @return newAngles (double[]) - angles in degrees
	 */
	public static double[] degrees(double[] angles) {
		double[] newAngles = new double[angles.length];
		for (int i=0; i < angles.length; i++) {
			newAngles[i] = degrees(angles[i]);
		}
		return newAngles;
	}

	/**
	 * Applies degrees() to a 2D array - why can't I just have numpy ;(
	 * 
	 * @param angles (double[][]) - angles in radians
	 * @return newAngles (double[][]) - angles in degrees
	 */
	public static double[][] degrees(double[][] angles) {
		double[][] newAngles = new double[angles.length][angles[0].length];
		for (int i=0; i < angles.length; i++) {
			newAngles[i] = degrees(angles[i]);
		}
		return newAngles;
	}

	/**
	 * Applies degrees() to a 2D ArrayList - why can't I just have numpy ;(
	 * 
	 * @param angles (ArrayList<double[]>) - angles in radians
	 * @return newAngles (ArrayList<double[]>) - angles in degrees
	 */
	public static ArrayList<double[]> degrees(ArrayList<double[]> angles) {
		ArrayList<double[]> newAngles = new ArrayList<double[]>(angles.size());
		for (int i=0; i < angles.size(); i++) {
			newAngles.add(degrees(angles.get(i)));
		}
		return newAngles;
	}

	public static double angularError(double[] angles1, double[] angles2) {
		return new R2Vector(angles1[0], angles1[1]).sub(new R2Vector(angles2[0], angles2[1])).mag();
	}
}
