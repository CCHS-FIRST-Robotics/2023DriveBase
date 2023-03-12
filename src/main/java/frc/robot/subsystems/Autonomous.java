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
 * TODO: Untested
 * 
 * @author xoth42
 */
public class Autonomous {

	/**
	 * @brief Generate a 2d cartesian trajectory (line/spline) and apply it to robot, with the endpoint given, "target," which reaches every point in interiorWaypoints, all relative to the drive base odometry.
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
	public static void updateTrajectory(MecaDrive m_drive, Pose2d target, ArrayList<Translation2d> interiorWaypoints)	{		
		// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.html
		// interior waypoints to-be-used, refrence https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(m_drive.getPose(), interiorWaypoints, target, m_drive.getTrajectoryConfig());
		m_drive.setCurrentTrajectory(trajectory);
	}

	/**
	 * @brief return chassis speeds (to be then scaled and applied to driving the robot) retrived from the trajectory of the robot and the time within the trajectory
	 * 
	 * (done in robot auton periodic) have mecadrive deal with setting trajectorytime (setTrajectoryTime) within autonomous (eg. knowing where to go, relative to robot's odometry). Probably best to be set within a periodic function.
	 * 
	 * @param m_drive (MecaDrive) drivebase, to get the trajectory, time, and pose 
	 * @return ChassisSpeeds - https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html
	 */
	private static ChassisSpeeds getChassisSpeeds(MecaDrive m_drive) {
		// get a (pose on spline) of the trajectory at "trajectoryTime" seconds relative to the beginning of the trajectory
		Trajectory.State goal = m_drive.getCurrentTrajectory().sample(m_drive.autonTimer.get()); 
		

		// return the chassis speeds (controller compares current pose of robot and goal to find the adequate speeds)
		return m_drive.getChassisController().calculate(m_drive.getPose(), goal);
	}

	/**
	 * @brief using the chassis speeds from the trajectory specified by the robot, drive the robot
	 * 
	 * TODO: verify functionality and test
	 * 
	 * @param m_drive MecaDrive drive base
	 */
	public static void applyChassisSpeeds(MecaDrive m_drive) {
		m_drive.drive(getChassisSpeeds(m_drive));
	}	

	/**
	 * @brief "linearized" in this case means a value that has been turned into a number from -1 to 1, relative to the given "max," which decides what value is associated with a linearized magnitude of 1.
	 * 
	 * TODO: check that this function is working correctly (double-check math)
	 * 
	 * @param velocity double meters/second
	 * @param maxSpeed double meters/second
	 * @return
	 */
	private static double linearizeVelocity(double velocity, double maxSpeed)	{
		// Speed is a scalar, (velocity is not scalar, negative matters) which means that maxSpeed has no direction, and thus a negative value in this case would imply a incorrect usage for maxSpeed.
		if (maxSpeed <= 0){
			throw new IllegalArgumentException("maxSpeed must be positive and greater than 0");
		}

		double linearizedVelocity; // from 1 to -1

		if (velocity == 0)	{
			linearizedVelocity = 0;
		} 
		else if (velocity < 0)	{
			// if velocity is in greater magnitude than maxSpeed, divide it by maxspeed so that it is in the range [0, 1]
			linearizedVelocity = Math.max(-1, velocity / maxSpeed);
		}
		else {
			linearizedVelocity = Math.min(1, velocity / maxSpeed);
		}
	
		return linearizedVelocity; 
	}

	/**
	 * @brief linearize velocity using the max velocity of the robot, overloaded (from Constants)
	 * @param velocity meters/second
	 * @return velocity from -1 to 1
	 */
	private static double linearizeVelocity(double velocity)	{
		return linearizeVelocity(velocity, Constants.maxVelocityMetersPerSecond);
	}
}

