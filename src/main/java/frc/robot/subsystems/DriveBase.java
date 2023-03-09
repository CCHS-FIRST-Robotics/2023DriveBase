package frc.robot.subsystems;
import java.lang.Math;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class DriveBase extends SubsystemBase {
	// used to scale speeds - 1 would be max speed, 0.5 would be half speed, etc.
	double speedMultiplier = Constants.DRIVE_STARTING_MULTIPLIER;

	// different modes
	enum Mode {
		DEFAULT_MODE,
		DEBUG_MODE,
		PID_TUNING_MODE,
		STOP_MODE
	}
	Mode currentMode = Mode.DEFAULT_MODE;

	// the motor to be activated during debug mode
	int debugEnabledMotor = 0;


	/*
	 * Autonomous variables and associated functions (trajectory & ramsete controller)
	 */
	private Trajectory currentTrajectory;
	private double currentTrajectoryTime; // seconds

	// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryConfig.html
	// needed for the updateTrajectory method in Autonomous
	protected TrajectoryConfig trajectoryConfig;
	
	// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
	// manages the forward, sideways, and rotational velocity of the robot (chassis speeds)
	private final RamseteController chassisController = new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA);


	/**
	 * Drive the robot with controller input
	 * 
	 * These are doubles on the interval [-1, 1] and come from the controller's
	 * analog sticks
	 * 
	 */
	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY){};


	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX){};

	public void drive(ChassisSpeeds chassisSpeeds){};
	/**
	 * Prints the controls of the current driving mode
	 */
	public abstract void printControlsOfCurrentMode();
	
	/**
	 * The speed bracket controls the multiplier for al the speeds
	 * So when you change it, lets say, to 1/2 speed, all movement will be at
	 * 1/2 speed
	 */
	public void increaseSpeedBracket() {
		speedMultiplier = Math.min(1, speedMultiplier + 0.4);
		// speedMultiplier = 1;
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void decreaseSpeedBracket() {
		// the min is 0.2 because below that the robot is unlikely to move
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.4);
		// speedMultiplier = .3;
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void turnOnDefaultMode() {
		if(currentMode == Mode.DEFAULT_MODE) return;
		currentMode = Mode.DEFAULT_MODE;
		System.out.println("********************************");
		System.out.println("Current Mode: DEFAULT Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
	}

	public void turnOnDebugMode() {
		if(currentMode == Mode.DEBUG_MODE) return;
        currentMode = Mode.DEBUG_MODE;
		System.out.println("********************************");
        System.out.println("Current Mode: DEBUG Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
    }

	/**
	 * should be overridden for actual functionality because depends on
	 * drive base specifc speeds (for tank drive, there is a left and right
	 * speed, whereas for mecanum there are 4)
	 */
	public void turnOnStopMode() {
		if(currentMode == Mode.STOP_MODE) return;
		currentMode = Mode.STOP_MODE;
		System.out.println("********************************");
		System.out.println("Current Mode: STOP Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
	}

	public void turnONPIDTuningMode() {
		if(currentMode == Mode.PID_TUNING_MODE) return;
		currentMode = Mode.PID_TUNING_MODE;
		System.out.println("********************************");
		System.out.println("Current Mode: PID TUNING Mode");
		System.out.println("********************************");
		printControlsOfCurrentMode();
	}

	/**
	 * Needs to be implemented in sub classes becuase there might
	 * be different numbers of motors
	 */
	public abstract void cycleMotor();
	
	// the following methods can be overridden for real functionality
	public void AButtonPressed() {}

	public void BButtonPressed() {}

	public void XButtonPressed() {}

	public void YButtonPressed() {}

	public void leftBumperPressed() {}

	public void rightBumperPressed() {}
	
	// Generated setters and getters (please use setters and getters)
	public Mode getCurrentMode() {
		return currentMode;
	}
	public void setCurrentMode(Mode currentMode) {
		this.currentMode = currentMode;
	}
	public int getDebugEnabledMotor() {
		return debugEnabledMotor;
	}
	public void setDebugEnabledMotor(int debugEnabledMotor) {
		this.debugEnabledMotor = debugEnabledMotor;
	}
	public Trajectory getCurrentTrajectory() {
		return currentTrajectory;
	}
	public void setCurrentTrajectory(Trajectory currentTrajectory) {
		this.currentTrajectory = currentTrajectory;
	}
	public TrajectoryConfig getTrajectoryConfig() {
		return trajectoryConfig;
	}
	public RamseteController getChassisController() {
		return chassisController;
	}	
	public double getCurrentTrajectoryTime()	{
		return currentTrajectoryTime;
	}
	public void resetCurrentTrajectoryTime()	{
		currentTrajectoryTime = 0;
	}
	public void incrementCurrentTrajectoryTime()	{
		currentTrajectoryTime += Constants.SECONDS_BETWEEN_CODE_PERIODS;
	}
}
