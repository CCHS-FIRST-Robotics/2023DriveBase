package frc.robot.subsystems;
import java.lang.Math;

public abstract class DriveBase {
	// used to scale speeds - 1 would be max speed, 0.5 would be half speed, etc.
	double speedMultiplier = 0.6;

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

	// sensors object
	public Sensors sensors = new Sensors();

	/**
	 * Drive the robot with controller input
	 * 
	 * These are doubles on the interval [-1, 1] and come from the controller's
	 * analog sticks
	 * 
	 */
	public abstract void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY);

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
}
