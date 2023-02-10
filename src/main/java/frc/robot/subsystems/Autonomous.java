package frc.robot.subsystems;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/**
 * @brief Contains functions for moving in autonomous mode, such as setting the robots trajectory, and getting speeds to set motors
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
	 * TODO: have mecadrive deal with setting trajectorytime (setTrajectoryTime) within autonomous (eg. knowing where to go, relative to robot's odometry)
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
}

