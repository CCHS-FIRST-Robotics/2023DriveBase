package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Rotation2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;

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
	RelativeEncoder sparkMaxEncoder;

	boolean debugMode = false;
    int debugEnabledWheel = 0;

	DifferentialDriveOdometry odometer;
	AHRS navx;

	double kP = 0.0001;
	double kI = 0.0;
	double kD = 0.0;
	PIDController pid = new PIDController(kP, kI, kD);

	double maxAngularVel = 28; // 24.0983606557377

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
		sparkMaxEncoder = leftSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, Constants.ENCODER_CPR);

		navx = new AHRS(SPI.Port.kMXP);
		odometer = new DifferentialDriveOdometry(new Rotation2d());
		resetPosition();
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



			// PID stuff

			// actual angular velocities
			double leftAngVel = sparkMaxEncoder.getVelocity();
			double rightAngVel = rightTalon.getSelectedSensorVelocity();

			// normalized (actual) angular velocities
			double normalLeftAngVel = leftAngVel / maxAngularVel;
			double normalRightAngVel = rightAngVel / maxAngularVel;

			// set the motors according to the PID
			leftVictor.set(pid.calculate(normalLeftAngVel, leftVel));
			leftSparkMax.set(pid.calculate(normalLeftAngVel, leftVel));

			rightTalon.set(pid.calculate(normalRightAngVel, rightVel));
			rightVictor.set(pid.calculate(normalRightAngVel, rightVel));

			

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
		updatePosition();
		// System.out.println(rightTalon.getSelectedSensorVelocity()); // clicks per 100ms
		// System.out.println(rightTalon.getSelectedSensorVelocity() * 10 * 60 / 4096);
		// System.out.println(sparkMaxEncoder.getVelocity()); // actual rpm
		// System.out.println("");




		// // Print angular velocity and motor use level
		// // get rotational speeds from the motors on each side
		// // divide by 60 to get rotations per second
		// double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;

		// if (Math.abs(leftRPS) > 0.1){
		// 	// // multiply by 10 because this is per 100ms - we want rps
		// 	double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;
		// 	System.out.println("Right Rps: " + rightRPS + "\nLeft RPS: " + leftRPS);
		// 	System.out.println("Right velocity from controller: " + rightVel + "\nLeft Velocity from controller: " + leftVel);
		// }
		 

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

	public void updateRobotVelocity() {
		
		// get rotational speeds from the motors on each side
		// divide by 60 to get rotations per second
		// double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;
		// // multiply by 10 because this is per 100ms - we want rps
		// double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;

		// // find linear velocities in m/s
		// double leftVel = (leftRPS * 2 * Math.PI) * Constants.TANK_WHEEL_RADIUS;
		// double rightVel = (rightRPS * 2 * Math.PI) * Constants.TANK_WHEEL_RADIUS;

		// currentSpeeds.leftMetersPerSecond = leftVel;
		// currentSpeeds.rightMetersPerSecond = rightVel;
	}

	public void getPosition()
	{

	}

	public void updatePosition()
	{
		// get the linear position of the left wheel and convert to meters
		double leftWheelPos = -1 * sparkMaxEncoder.getPosition() * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the linear position of the right wheel and convert to meters
		double rightWheelPos = rightTalon.getSelectedSensorPosition() / Constants.ENCODER_CPR * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the angle of the robot
		double theta = navx.getAngle() / 180 * Math.PI;
		Rotation2d rot = new Rotation2d(theta);
		odometer.update(rot, leftWheelPos, rightWheelPos);
	}

	public void resetPosition()
	{
		// reset the sensors to zero the position
		navx.reset();
		navx.resetDisplacement();
		sparkMaxEncoder.setPosition(0);
		rightTalon.setSelectedSensorPosition(0);
		odometer.resetPosition(new Pose2d(), new Rotation2d());
	}

	public void printPosition()
	{
		Pose2d posFromWheelDisplacement = odometer.getPoseMeters();
		System.out.println("Position from odometer and wheel: ");
		System.out.println("X: " + posFromWheelDisplacement.getX() + " Y: " + posFromWheelDisplacement.getY() + " rotation: " + posFromWheelDisplacement.getRotation());

		System.out.println("\nPosition info from navx: ");
		System.out.println("X: " + navx.getDisplacementX() + " Y: " + navx.getDisplacementY() + " rotation: " + navx.getAngle());
		System.out.println();

		// get the linear position of the left wheel and convert to meters
		double leftWheelPos = -1 * sparkMaxEncoder.getPosition() * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;

		// get the linear position of the right wheel and convert to meters
		double rightWheelPos = rightTalon.getSelectedSensorPosition() / Constants.ENCODER_CPR * 2 * Math.PI * Constants.TANK_WHEEL_RADIUS;
		System.out.println("Right Pos: " + rightWheelPos + " left pos: " + leftWheelPos);


		// Print angular velocity and motor use level
		// get rotational speeds from the motors on each side
		// divide by 60 to get rotations per second
		double leftRPS = -1 * sparkMaxEncoder.getVelocity() / 60;
		// // multiply by 10 because this is per 100ms - we want rps
		double rightRPS = rightTalon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_CPR;
		System.out.println("Right Rps: " + rightRPS + "\nLeft RPS: " + leftRPS);

	}
	
}
