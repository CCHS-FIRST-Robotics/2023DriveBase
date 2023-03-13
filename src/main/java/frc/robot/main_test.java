package frc.robot;

import frc.robot.utils.*;
import java.util.ArrayList;

public class main_test {
	public static void main(String[] args) {
		QuadraticProfile qp = new QuadraticProfile(.02);
		
		ArrayList<double[]> trajectory = getTrajectory(0.3, 0.7);
//		for(int i = 0; i < trajectory.size(); i++) {
//			System.out.println("(" + trajectory.get(i)[0] + ", " + trajectory.get(i)[1] + ")");
//		}
		
		printXTrajectory(trajectory);
		
		for(int i = 0; i < 5; i++) {
			System.out.println();
		}
		printYTrajectory(trajectory);
	}
	
	public static ArrayList<double[]> getTrajectory(double x, double y) {
		// USE WRIST JOINT POS SINCE IK CAN'T HANDLE WRIST YET
		
		double[] jointAngles = new double[2];
		jointAngles[0] = 45;
		jointAngles[1] = -90;
		
		double[] current_pos = Kinematics.forwardKinematics(jointAngles[0], jointAngles[1]);
		
		double wristAngle = jointAngles[0] + jointAngles[1] + 90 * 0;
		

		ArrayList<double[]> trajectory = new QuadraticProfile().getSetPoints(
			new R2Vector(current_pos[0], current_pos[1]),  // initial pos
			jointAngles, // initial angles
			new R2Vector(x, y), // goal pos
			wristAngle, // wrist angle
			Constants.ARM_MAX_SPEED,
			Constants.ARM_MAX_ACCELERATION
		);

		return trajectory;
	}
	
	public static void printXTrajectory(ArrayList<double[]> trajectory) {
		for(int i = 0; i < trajectory.size(); i++) {
			System.out.println("(" + (0.02 * i) + ", " + trajectory.get(i)[0] + ")");
		}
	}
	
	public static void printYTrajectory(ArrayList<double[]> trajectory) {
		for(int i = 0; i < trajectory.size(); i++) {
			System.out.println("(" + (0.02 * i) + ", " + trajectory.get(i)[1] + ")");
		}
	}
	
}
