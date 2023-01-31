package frc.robot.subsystems;
import java.lang.Math;

public abstract class DriveBase {
	// used to scale speeds - 1 would be max speed, 0.5 would be half speed, etc.
	double speedMultiplier = 0.6;

	// different modes
	String currentMode;
	static final String DEFAULT_MODE = "DEFAULT"; // regular driving
	static final String DEBUG_MODE = "DEBUG"; // spin only one motor at a time
	static final String PID_TUNING_MODE = "PIDTUNING"; // tune PID constants
	static final String STOP_MODE = "STOP"; // stop the robot

	// the motor to be activated during debug mode
	int debugEnabledMotor = 0;

	/**
	 * Drive the robot with controller input
	 * 
	 * These are doubles on the interval [-1, 1] and come from the controller's
	 * analog sticks
	 * 
	 */
	public abstract void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY);

	public abstract void driveWPI(double leftAnalogX, double leftAnalogY, double rightAnalogX);

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
		speedMultiplier = Math.min(1, speedMultiplier + 0.1);
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void decreaseSpeedBracket() {
		// the min is 0.2 because below that the robot is unlikely to move
		speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
		System.out.println("Current speed multiplier: " + speedMultiplier);
	}

	public void turnOnDefaultMode() {
		if(currentMode.equals(DEFAULT_MODE)) return;
		currentMode = DEFAULT_MODE;
		System.out.println("Current Mode: DEFAULT Mode");
		printControlsOfCurrentMode();
	}

	public void turnOnDebugMode() {
		if(currentMode.equals(DEBUG_MODE)) return;
        currentMode = DEBUG_MODE;
        System.out.println("Current Mode: DEBUG Mode");
		printControlsOfCurrentMode();
    }

	/**
	 * should be overridden for actual functionality because depends on
	 * drive base specifc speeds (for tank drive, there is a left and right
	 * speed, whereas for mecanum there are 4)
	 */
	public void turnOnStopMode() {
		if(currentMode.equals(STOP_MODE)) return;
		currentMode = STOP_MODE;
		System.out.println("Current Mode: STOP Mode");
		printControlsOfCurrentMode();
	}

	public void turnONPIDTuningMode() {
		if(currentMode.equals(PID_TUNING_MODE)) return;
		currentMode = PID_TUNING_MODE;
		System.out.println("Current Mode: PID TUNING Mode");
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
}
