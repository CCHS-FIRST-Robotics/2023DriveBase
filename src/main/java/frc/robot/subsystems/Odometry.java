package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Odometry extends MecanumDriveOdometry{
    IMU imu;
    
    public Odometry(
            MecanumDriveKinematics kinematics,
            Rotation2d gyroAngle,
            Pose2d initialPoseMeters,
            IMU imu) {
        super(kinematics, gyroAngle, new MecanumDriveWheelPositions(), initialPoseMeters);
        this.imu = imu;
    }

    public Odometry(
            MecanumDriveKinematics kinematics,
            Rotation2d gyroAngle,
            IMU imu) {
        this(kinematics, gyroAngle, new Pose2d(), imu);
    }

    /**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	private Pose2d getPose() {
		return getPoseMeters();
	}

	// TODO: create method that returns wheel speeds of the robot
	// TODO: create mthod that allows control of wheels with voltages
	// TODO: method of averages of encoder distances

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	private void resetOdometry(Pose2d pose) {
		//resetEncoders();
		resetPosition(new Rotation2d(Math.toRadians(imu.getHeading())), getWheelPositions(), pose);
	}

    /**
	 * Returns the total distances measured by each motor
	 * 
	 * @return wheel positions
	 */
	private MecanumDriveWheelPositions getWheelPositions() {

		// TODO: determine whether should use absolute position or just position

		return new MecanumDriveWheelPositions(flFalconSensor.getIntegratedSensorAbsolutePosition(), 
												frFalconSensor.getIntegratedSensorAbsolutePosition(), 
												rlFalconSensor.getIntegratedSensorAbsolutePosition(),
												rrFalconSensor.getIntegratedSensorAbsolutePosition());
	}

}
