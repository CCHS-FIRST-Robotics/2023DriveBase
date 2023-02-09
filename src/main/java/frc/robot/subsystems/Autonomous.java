package frc.robot.subsystems;
import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

/**
 * Contains functions for moving in autonomous mode, such as setting the robots trajectory, and getting speeds to set motors
 */
public class Autonomous {

	/**
	 * @brief Generate a trajectory (line/spline) and apply it to robot, with the endpoint given, "target," which reaches every point in interiorWaypoints. Drive base needed for start and config. All "points" are on the cartesian plane created with odometry.
	 * 
	 *  For reference, see link
	 *  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
	 * 
	 * @param m_drive drive base, for start pose and trajectory config (MecaDrive)
	 * @param target ending location (Pose2d)
	 * @param interiorWaypoints the points that the trajectory must reach (ArrayList<Translation2d>)
	 */
	public void updateTrajectory(MecaDrive m_drive, Pose2d target, ArrayList<Translation2d> interiorWaypoints)	{		
		// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.html
		// interior waypoints to-be-used, refrence https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(m_drive.getPose(), interiorWaypoints, target, m_drive.getTrajectoryConfig());
		m_drive.setTrajectory(trajectory);
	}

	// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
	RamseteController robotController = new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA);

	/**
	 * @brief return chassis speeds (to be then scaled and applied to driving the robot) retrived from the trajectory of the robot and the time within the trajectory
	 * 
	 * @param m_drive (MecaDrive) drivebase, to get the trajectory, time, and pose 
	 * @return ChassisSpeeds - https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/ChassisSpeeds.html
	 */
	public ChassisSpeeds getChassisSpeeds(MecaDrive m_drive) {
		// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
		// TODO: have mecadrive deal with setting trajectorytime (setTrajectoryTime) within autonomous

		// get the trajectory at "trajectoryTime" seconds from the beginning
		Trajectory.State goal = m_drive.getTrajectory().sample(m_drive.getTrajectoryTime()); 

		// return the chassis speeds 
		return robotController.calculate(m_drive.getPose(), goal);
	}
}

