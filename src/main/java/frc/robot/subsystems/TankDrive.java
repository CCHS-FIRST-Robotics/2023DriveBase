package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Manages the tank drive base
 * 
 */
public class TankDrive {
	
	// between 0 and 1 - 1 would be full max speed, 0.5 would be half speed, etc
	double maxSpeed = 0.5;

	// scales the rotational speed of the robot, controlled by the right analog
	double rotationalSpeedMultiplier = 0.75;

	// TalonSRX leftMotor1, leftMotor2, rightMotor1, rightMotor2;
	TalonSRX leftTalon, rightTalon;
	VictorSPX leftVictor, rightVictor;
	CANSparkMax leftSparkMax;

	boolean debugMode = false;
    int debugEnabledWheel = 0;

	/**
	 * Constructor for TankDrive Class
	 * 
	 * @param leftMotorPort
	 * @param rightMotorPort
	 */
	public TankDrive(int leftTalonPort, int leftVictorPort,
					 int rightTalonPort, int rightVictorPort) {
		// leftMotor1 = new TalonSRX(leftMotorPort1);
		// leftMotor2 = new TalonSRX(leftMotorPort2);
		// rightMotor1 = new TalonSRX(rightMotorPort1);
		// rightMotor2 = new TalonSRX(rightMotorPort2);
		// leftTalon = new TalonSRX(leftTalonPort);
		leftVictor = new VictorSPX(leftVictorPort);
		leftSparkMax = new CANSparkMax(leftTalonPort, CANSparkMaxLowLevel.MotorType.kBrushed);
		rightTalon = new TalonSRX(rightTalonPort);
		rightVictor = new VictorSPX(rightVictorPort);
	}
	
	/**
	 * Drive the robot tank base from controller input 
	 * 
	 * These are doubles on the interval [-1,1], and come from the controller's 
	 * 	analog sticks (circle spinny things)
	 * @param leftAnalogX 
	 * @param leftAnalogY 
	 * @param rightAnalogX 
	 * @param rightAnalogY 
	 */
	public void drive(double leftAnalogX, double leftAnalogY,
					  double rightAnalogX, double rightAnalogY) {

		/**
		 * we only care about left Y and right X
		 * left Y is average velocity
		 * right X is velocity difference between wheels
		*/

		// TODO: test multiplying by 2 (to have a larger difference between wheel speeds)
		double x = rightAnalogX; 
		
		double y = leftAnalogY;

		// make sure that both velocities are in [-1, 1]
		double preScaledLeftVel = y - x * rotationalSpeedMultiplier;
		double preScaledRightVel = y + x * rotationalSpeedMultiplier;
		double scaleFactor = 1 / Math.max(Math.max(Math.abs(preScaledLeftVel), Math.abs(preScaledRightVel)), 1);
		
		// scale to what the controller asks
		double leftVel = preScaledLeftVel * scaleFactor * maxSpeed;
		double rightVel = preScaledRightVel * scaleFactor * maxSpeed;
		
		// // leftTalon.set(ControlMode.PercentOutput, leftVel);
		// leftVictor.set(ControlMode.PercentOutput, leftVel);
		// leftSparkMax.set(leftVel);
		// rightTalon.set(ControlMode.PercentOutput, rightVel);
		// rightVictor.set(ControlMode.PercentOutput, rightVel);
		// set the motor speeds
        if(!debugMode) {
			leftVictor.set(ControlMode.PercentOutput, -1 *leftVel);
			leftSparkMax.set(leftVel);
			rightTalon.set(ControlMode.PercentOutput, rightVel);
			rightVictor.set(ControlMode.PercentOutput, rightVel);

			// System.out.println("leftvel: " + leftVel);
			// System.out.println("rightvel: " + rightVel);
			// System.out.println(" ");
        } else {
            switch (debugEnabledWheel){
                case 0:
                    leftVictor.set(ControlMode.PercentOutput, -1 * leftVel);
                    break;
                case 1:
                    rightVictor.set(ControlMode.PercentOutput, rightVel);
                    break;
                case 2:
                    rightTalon.set(ControlMode.PercentOutput, rightVel);
                    break;
                case 3:
                    leftSparkMax.set(leftVel);
                    break;
            }
        }
	}

	/**
	 * The speed bracket controls the multiplier for all the speeds 
	 * So when you change it, lets say, to 1/2 speed, all directions will be
	 * 	at 1/2 speed
	 */
	public void increaseSpeedBracket() {
		maxSpeed = Math.min(0.8, maxSpeed + 0.1);
	}
  
	public void decreaseSpeedBracket() {
		maxSpeed = Math.max(0.2, maxSpeed - 0.1);
	}

	public void toggleDebugMode() {
        debugMode = !debugMode;
        System.out.println("Debug Mode: " + debugMode);
    }

    public void cycleWheelDebugMode() {
        debugEnabledWheel++;
        debugEnabledWheel %= 4;
        System.out.println("Current Wheel: " + debugEnabledWheel);
    }
}
