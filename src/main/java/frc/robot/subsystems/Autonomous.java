package frc.robot.subsystems;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

/**
 * @brief Contains functions for moving in autonomous mode, such as setting the robots trajectory, and getting speeds to set motors
 * 
 * todo: Untested
 * 
 * @author xoth42
 */
public class Autonomous {

	/**
	 * @brief Generate a trajectory (line/spline) and apply it to robot, with the endpoint given, "target," which reaches every point in interiorWaypoints. Drive base needed for start and config. All "points" are on the cartesian plane created with odometry.
	 * 
	 *  For reference, see link
	 *  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
	 * 
	 * @param m_drive drive base, for start pose and trajectory config (MecaDrive)
	 * m_drive could be DriveBase instead, but there is no getPose() for tank drive, same thing with getChassisSpeeds
	 * 
	 * @param target ending location (Pose2d)
	 * @param interiorWaypoints the points that the trajectory must reach (ArrayList<Translation2d>)
	 */
	public void updateTrajectory(MecaDrive m_drive, Pose2d target, ArrayList<Translation2d> interiorWaypoints)	{		
		// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.html
		// interior waypoints to-be-used, refrence https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(m_drive.getPose(), interiorWaypoints, target, m_drive.getTrajectoryConfig());
		m_drive.setCurrentTrajectory(trajectory);
	}

	/**
	 * @brief return chassis speeds (to be then scaled and applied to driving the robot) retrived from the trajectory of the robot and the time within the trajectory
	 * 
	 * TODO: have mecadrive deal with setting trajectorytime (setTrajectoryTime) within autonomous (eg. knowing where to go, relative to robot's odometry). Probably best to be set within a periodic function.
	 * 
	 * @param m_drive (MecaDrive) drivebase, to get the trajectory, time, and pose 
	 * @return ChassisSpeeds - https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html
	 */
	public ChassisSpeeds getChassisSpeeds(MecaDrive m_drive) {
		// get a state of the trajectory at "trajectoryTime" seconds relative to the beginning of the trajectory
		Trajectory.State goal = m_drive.getCurrentTrajectory().sample(m_drive.getCurrentTrajectoryTime()); 

		// return the chassis speeds (controller compares current pose of robot and goal to find the adequate speeds)
		return m_drive.getChassisController().calculate(m_drive.getPose(), goal);
	}

	/**
	 * @brief using the chassis speeds from the trajectory specified by the robot, drive the robot
	 * 
	 * Todo: verify functionality and test
	 * 
	 * @param m_drive MecaDrive drive base
	 */
	public void applyChassisSpeeds(MecaDrive m_drive)	{
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds(m_drive);
		
		m_drive.drive(linearizeSpeed(currentChassisSpeeds.vxMetersPerSecond), linearizeSpeed(currentChassisSpeeds.vyMetersPerSecond), linearizeSpeed(currentChassisSpeeds.omegaRadiansPerSecond, Constants.DRIVE_MAX_ANGULAR_VELOCITY));
	}

	/**
	 * @brief given input speed (double), return, if greater than the max speed, 1 or -1. Else, return a value between 1 and -1 equal to the given speed divided by the maximum speed of the robot (Constants.maxVelocityMetersPerSecond)
	 * 
	 * TODO: add a value for maxVelocityMetersPerSecond to Constants.java
	 * TODO: check that this function is working correctly (double-check math)
	 * 
	 * @param speed double meters/second
	 * @return
	 */
	private double linearizeSpeed(double speed)	{
		double speedIsNegative = speed < 0 ? 1d : 0d; // if negative, value is a double, 1 else 0
		double absSpeed = Math.abs(speed);

		// set speed to max if past max (just in case, also if equal) or set val to speed / max speed (from 0 to 1)
		double newSpeed = absSpeed >= Constants.maxVelocityMetersPerSecond ? Constants.maxVelocityMetersPerSecond : absSpeed / Constants.maxVelocityMetersPerSecond;

		// (apply if the speed was negative to begin with)
		// if speedIsNegative, newSpeed will be multiplied by -1^1=-1, else by -1^0=1
		return Math.pow(-1, speedIsNegative) * newSpeed; // between -1 and 1
	}

	/**
	 * Overloaded version of linearizeSpeed(double speed) that adds a param for max speed when Constants.maxVelocityMetersPerSecond is not what should be used (when using rotational velocity)
	 * @brief given input speed (double), return, if greater than the maxSpeed, 1 or -1. Else, return a value between 1 and -1 equal to the given speed divided by the maximum speed (maxSpeed)
	 * 
	 * TODO: check that this function is working correctly (double-check math)
	 * 
	 * @param speed double meters/second
	 * @param maxSpeed double meters/second
	 * @return
	 */
	private double linearizeSpeed(double speed, double maxSpeed)	{
		double speedIsNegative = speed < 0 ? 1d : 0d; // if negative, value is a double, 1 else 0
		double absSpeed = Math.abs(speed);

		// set speed to max if past max (just in case, also if equal) or set val to speed / max speed (from 0 to 1)
		double newSpeed = absSpeed >= maxSpeed ? maxSpeed : absSpeed / maxSpeed;

		// (apply if the speed was negative to begin with)
		// if speedIsNegative, newSpeed will be multiplied by -1^1=-1, else by -1^0=1
		return Math.pow(-1, speedIsNegative) * newSpeed; // between -1 and 1
	}
}

