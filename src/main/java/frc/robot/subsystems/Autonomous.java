package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class Autonomous {
	PIDController xPID; // for x movement
	PIDController yPID; // for y movement

	MecaDrive mDrive;

	double maxOutput;

	Pose2d target;

	// tolerances in meters
	static final double X_TOLERANCE = 0.02;
	static final double Y_TOLERANCE = 0.02;

	Autonomous(MecaDrive mDrive) {
		this.mDrive = mDrive;

		xPID = new PIDController(Constants.X_PID[0], Constants.X_PID[1], Constants.X_PID[2]);
		yPID = new PIDController(Constants.Y_PID[0], Constants.Y_PID[1], Constants.Y_PID[2]);
	}

	/**
	 * set the destination for the robot to drive to
	 * 
	 * @param dest the desired position (absolute, not relative to the robot)
	 */
	public void setDestination(Pose2d dest) {
		target = dest;
	}

	/**
	 * set the maximum pid output for x and y velocity (in terms of percent output)
	 * 
	 * @param maxOutput the magnitude of the maximum output desired
	 */
	public void limitOutput(double maxOutput) {
		this.maxOutput = maxOutput;
	}

	/**
	 * drive the robot toward the setpoint
	 */
	public void drive() {
		if (atDestination()) mDrive.driveStraight(0, 0, 0, true);

		double currentX = mDrive.getPoseX();
		double currentY = mDrive.getPoseY();

		// TODO: may need to add feedforward to overcome resistive forces
		double xVel = MathUtil.clamp(xPID.calculate(currentX, target.getX()), -maxOutput, maxOutput);
		double yVel = MathUtil.clamp(yPID.calculate(currentY, target.getY()), -maxOutput, maxOutput);

		// TODO: not sure if this will be on [0, 360] or [-180, 180] or something
		mDrive.headingSetPoint = target.getRotation().getDegrees(); // heading setpoint will take care of rotational velocity

		// TODO: check if this should be field oriented or not
		mDrive.driveStraight(xVel, yVel, 0, true);
	}

	/**
	 * checks if the robot is close enough to the destination
	 * WARNING: DOES NOT CHECK IF HEADING IS CLOSE BECUASE HEADING CONTROL IS DONE IN THE driveStraight() METHOD
	 * 
	 * @return true if x and y are close to destination, false otherwise
	 */
	public boolean atDestination() {
		if (Math.abs(mDrive.getPoseX() - target.getX()) > X_TOLERANCE) return false;
		if (Math.abs(mDrive.getPoseY() - target.getY()) > Y_TOLERANCE) return false;
		return true;
	}
}

