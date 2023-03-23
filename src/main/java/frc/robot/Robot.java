// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.ConnectionStrategy;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.*;
import frc.robot.utils.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	enum AutonStates {
		MoveArmToScore,
		WaitForArmPre,
		WaitForArm,
		OpenGrabber,
		WaitForGrabber,
		FoldArm,
		WaitForFoldedArm,
		DriveInit,
		Drive,
		Balance,
		BalanceAlternate,
		FlipWrist,
		WaitForWrist
	};
	
	AutonStates autonState = AutonStates.MoveArmToScore;

	enum TeleopStates {
		rampAssistedBalance,
		rampHold,
		assistedAlignLime,
        assistedAlignZED,
		manual
	}

	TeleopStates teleopState = TeleopStates.manual;

	// state of whether we're picking up cone or cube
	boolean cone = true;
	boolean cube = false;
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	private Controller xboxController = new Controller(Constants.XBOX_CONTROLLER_PORT, 5);
	private MonkeyController monkeyController = new MonkeyController(Constants.XBOX_CONTROLLER_ALTERNATE_PORT, 5);

	DigitalInput limitSwitch = new DigitalInput(0);
	DigitalOutput ledOutput = new DigitalOutput(5);

	Grabber claw = new Grabber(Constants.CLAW_FORWARD_NUM, Constants.CLAW_BACKWARD_NUM, 
								Constants.WRIST_FORWARD_NUM, Constants.WRIST_BACKWARD_NUM);
	Arm arm = new Arm(Constants.SHOULDER_TALON_ID, Constants.SHOULDER_FALCON_ID, Constants.ELBOW_TALON_ID, Constants.ELBOW_FALCON_ID, limitSwitch, claw);

	Limelight limelight = new Limelight();
	ZED zed = new ZED();
	IMU imu = new IMU(Constants.ANALOG_GYRO_PORT);

//   VideoSink server;
//   UsbCamera camera0, camera1;
	
	BetterShuffleboard smartdash = new BetterShuffleboard();

	private MecaDrive driveBase = new MecaDrive(Constants.FL_TALON_ID, Constants.FR_TALON_ID, Constants.RL_TALON_ID, Constants.RR_TALON_ID, imu);
	
	boolean autonomousIsMoving = true;

	double pidTuningAlpha;
	double pidTuningBeta;

	boolean fieldOriented = true;
	boolean autoClaw = false;

	double test = 0;
	long counter = 0; // for calling functions every n loops

	public Robot() {
		addPeriodic(() -> imu.updateGyro(), .001);
	}

	long autonCounter = 0;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		m_chooser.addOption("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// driveBase.mDrive.setSafetyEnabled(false);

    	// CAMERAS + CONFIG
		// camera0 = CameraServer.startAutomaticCapture(0);
		// camera1 = CameraServer.startAutomaticCapture(1);
    	// server = CameraServer.getServer();

		// camera0.setResolution(640, 480);
		// camera1.setResolution(640, 480);
		// camera0.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		// camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

		//limelight.printVal();
		//limelight.smartDash();

		// tank drive initialization
		// driveBase = createTankDrive();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 * 
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {

		
		driveBase.updateOdometry();

		ledOutput.set(cone);

		// SmartDashboard.putNumber("test", test);
		// limelight.test();
		// System.out.println("hello wo5rld");

		// test += 1;

		if (counter % 10 == 0) {
			smartdash.pushDashboard(limelight, imu, driveBase, zed);
		}
		counter++;
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different
	 * autonomous modes using the dashboard. The sendable chooser code works with the Java
	 * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to the switch structure
	 * below with additional strings. If using the SendableChooser make sure to add them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		// disable safety because we are not driving with this in autonomous
		driveBase.mDrive.setSafetyEnabled(false);

		driveBase.clearOdom();

		// create an example trajectory		
		// Pose2d current = driveBase.getPose();
		// start with a small displacement ( + 1)
		// Pose2d target = new Pose2d(current.getX(), current.getY() + 1, current.getRotation().plus(new Rotation2d(0)));
		// Autonomous.updateTrajectory(driveBase, target, new ArrayList<Translation2d>());
		// driveBase.resetCurrentTrajectoryTime();
		
		// System.out.println("zeroing counter");
		autonCounter = 0;
		autonState = AutonStates.MoveArmToScore;

		// set up timer for autonomous
		// driveBase.autonTimer = new Timer();
		// driveBase.autonTimer.start();
		driveBase.setMotorsNeutralMode(NeutralMode.Brake);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// // switch (m_autoSelected) {
		// //   case kCustomAuto:
		// //     // Put custom auto code here
		// //     break;
		// //   case kDefaultAuto:
		// //   default:
		// //     break;
			// // // Default code
		// // }
		// // stop the robot after a while
		// // if (auton_counter > 100) autonomousIsMoving = false;
			// // this if statement is kind of irrelevant because we have no way to change this bool
		// // because all controller input is ignored during autonomous
			// if (autonomousIsMoving){
			// 	// increase the current time, because autonomous trajectories need a time (each period takes the same time)
			// 	// driveBase.incrementCurrentTrajectoryTime(); // so add it up
			// 	// tell the autonomous system to use it's trajectory from the drivebase to drive the robot
		//   // Autonomous.applyChassisSpeeds(driveBase);

		//   // +x is forward, +y is right, +z is clockwise as viewed from above
		//   // driveBase.mDrive.driveCartesian(0, 0, -Math.PI/16);
		
		//   // +x is forward, +y is left
		//   // driveBase.drive(new ChassisSpeeds(.25, 0, 0));

		//   // it seems to be driving about 500 click/100ms too fast??
		//   driveBase.frontLeftMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS); // should be about 25%
		//   driveBase.frontRightMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS); // should be about 25%
		//   driveBase.rearLeftMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS); // should be about 25%
		//   driveBase.rearRightMotor.set(ControlMode.Position, Constants.METERS_TO_FALCON_CLICKS); // should be about 25%
		//   driveBase.printVelocity();
		//   // System.out.println(driveBase.getOdomHeading());
			// }
		// else {
		//   // System.out.println("Not moving");
		//   driveBase.drive(0, 0, 0);
		// }
			// auton_counter++;

		arm.run();

		switch (autonState) {
			case MoveArmToScore:
				System.out.println("Moving Arm");
				arm.setEndEffector(Constants.ArmFixedPosition.CONE_HIGHER_PRE_POS);
				autonState = AutonStates.WaitForArmPre;
				break;
			case WaitForArmPre:
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION) {
					arm.setEndEffector(Constants.ArmFixedPosition.CONE_HIGHER);
					autonState = AutonStates.WaitForArm;
				}
				break;
			case WaitForArm:
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION)
					autonState = AutonStates.OpenGrabber;
				break;
			case FlipWrist:
				claw.wristBack();
				autonState = AutonStates.WaitForWrist;
				autonCounter = 150;
				break;
			case WaitForWrist:
				autonCounter--;
				if (autonCounter == 0)
					autonState = AutonStates.OpenGrabber;
				break;
			case OpenGrabber:
				claw.clawBack();
				autonCounter = 30;
				autonState = AutonStates.WaitForGrabber;
				break;
			case WaitForGrabber:
				autonCounter--;
				if (autonCounter == 0)
					autonState = AutonStates.FoldArm;
				break;
			case FoldArm:
				System.out.println("FOLDING ARM");
				arm.setEndEffector(Constants.ArmFixedPosition.NEUTRAL);
				autonState = AutonStates.Drive;
				break;
			case WaitForFoldedArm:
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION)
					autonState = AutonStates.Drive;
				break;
			case DriveInit:
				break;
			case Drive:
				// zero all the encoders so it goes straight
				// driveBase.clearOdom();
				if (Constants.ROBOT_START_CENTER_FIELD) {
					// drives backwards to the ramp 
					// driveBase.setPosition(-3);
					if (Math.abs(imu.getPitch()) > 7) {
						System.out.println("test");
						autonState = AutonStates.Balance;
						break;
					}
					driveBase.drive(0, -.4, 0, false);
				} else {
					// drive backwards outside the community (-4 works)
					// -1.6 is good for ramp
					driveBase.setPosition(-4);
				}
				break;
			case Balance:
				driveBase.rampAutoBalance();
				break;
			case BalanceAlternate:
				double pitch = Math.abs(imu.getPitch());
				double sgn = pitch / imu.getPitch();
				if (pitch < 2) {
					driveBase.drive(0, 0,  0, false);
				} else if (pitch < 7) {
					driveBase.drive(0, -sgn * -.17,  0, false);
				} else {
					driveBase.drive(0, -sgn * -0.25, 0, false);
				}
				break;
		}
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		smartdash.updateControllerExponents();
		smartdash.updatePIDConstants(arm);
		driveBase.setMotorsNeutralMode(NeutralMode.Coast);

		// arm.setNeutralPostion();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		// check for button/bumper presses
		checkForButtonPresses();

		double leftX = xboxController.getLeftX();
		double leftY = xboxController.getLeftY();
		double rightX = xboxController.getRightX();

		if (!Constants.isZero(leftX) || !Constants.isZero(leftY) || !Constants.isZero(rightX)) {
			if (teleopState != TeleopStates.assistedAlignLime || !Constants.isZero(leftX))
				teleopState = TeleopStates.manual;
		}

		/*
		* DRIVE CODE
		*/
		double heading;
		switch(teleopState) {
			case rampAssistedBalance:
				driveBase.rampAutoBalance();
				break;
			case rampHold:
				driveBase.rampHold();
				// driveBase.holdPosition();
				
				// driveBase.setMotorsNeutralMode();
				break;

            case assistedAlignLime:
                double xOffset = limelight.getX(0) - 11.1;
                heading = imu.getHeading();
				if (Constants.isZero(xOffset)) {
					break;
				}
				// -10*(heading / 180)
				// System.out.println(xOffset/180);
                driveBase.drive(xOffset/180 * 15, leftY, -10*(heading / 180), false);
                break;

			case assistedAlignZED:
				ZED.Position zedPos = ZED.Position.CONE;
				if (cube) zedPos = ZED.Position.CUBE;

				// handle the case where no tags are found
				if (zed.getAprilTagId() == -1) {
					break;
				}

				double[] pos = zed.getAprilTagPos(zedPos);
				double dx = pos[0];
				double dy = pos[1];
				heading = imu.getHeading();
				// if (counter % 10 == 0)
				// 	System.out.println(dx * .5);
				double sign = Math.abs(dx) / dx;
				double controlInput = sign * Math.max(.2, Math.abs(dx * 4));
				driveBase.drive(controlInput, 0, 0, false);
				break;
			case manual:
				driveBase.drive(leftX, leftY, rightX * 0.5, fieldOriented);
				// the following doesn't work (there seems to be issues with the velocity control mode)
				// driveBase.drive(new ChassisSpeeds(leftY * Constants.DRIVE_MAX_X_VELOCITY,
				// 								  leftX * Constants.DRIVE_MAX_Y_VELOCITY,
				// 								  rightX * Constants.DRIVE_MAX_ANGULAR_VELOCITY));
				break;
		}

		/*
		 * ARM CODE
		 */

		// System.out.println("Alpha:" + arm.getShoulderAngle());
		// System.out.println("Beta:" + arm.getElbowAngle());
		// System.out.println("\n\n");

		// System.out.println(arm.getCurrentMode());

		// Arm code is self-contained, only need to call run() and the state machine inside Arm will handle the rest
    	// System.out.println(monkeyController.getPrimaryX());
		arm.run(-monkeyController.getPrimaryX(), monkeyController.getPrimaryY(), monkeyController.getSecondaryX(), monkeyController.getSecondaryY());

		// arm.getElbowRawAngle();

		// arm.setShoulder(pidTuningAlpha);
		// arm.setElbow(pidTuningBeta);

		// arm.moveArm(xboxController.getLeftX(), xboxController.getLeftY());


		// arm.setEndEffector(1, 1, arm.getWristAngle());


		// if (xboxController.getRightBumperPressed()) {
		//   // driveBase.increaseSpeedBracket();
		// } else if (xboxController.getAButtonPressed()) {
		//   // driveBase.cycleMotor();
		//   arm.setEndEffector(1.6, 1);
		// } else {
		//   arm.setEndEffector(.3, .4);
		// }

		// arm.setEndEffector(1.55, 1.2);

		// arm.moveArm(.3 * xboxController.getLeftX(), .3 * xboxController.getLeftY());


		if (counter % 20 == 0) {
			// System.out.println(pidTuningAlpha);
			// System.out.println(arm.getShoulderAngle());
			// System.out.println("SHOULDER FF" + arm.getShoulderFeedforward());
			// System.out.println("ELBOW FF:" + arm.getElbowFeedforward());

			// double angles[] = Kinematics.positionInverseKinematics(1, 1, 0);
			// System.out.println("SATRT OF THIGN");
			// System.out.println(Math.toDegrees(angles[0]));
			// System.out.println(Math.toDegrees(angles[1]));
			// System.out.println("END OF THIGN");

			// System.out.println(arm.getShoulderRawAngle());
			// Kinematics.positionInverseKinematics(1, 1, arm.getWristAngle());

			// System.out.println("SHOULDER: " + arm.getShoulderAngle());
			// System.out.println("ELBOW: " + arm.getElbowAngle());

			if (arm.shouldMotorStop()) {
				// System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
			}

			// System.out.println(limitSwitch.get());
			// System.out.println("test working");

			smartdash.putNumber("SHOULDER ENCODER", arm.getShoulderAngle());
			smartdash.putNumber("ELBOW ENCODER", arm.getElbowAngle());

			smartdash.putNumber("DESIRED ALPHA", pidTuningAlpha);
			smartdash.putNumber("DESIRED BETA", pidTuningBeta);

			// System.out.println(arm.getShoulderFeedforward());

			// System.out.println("SHOULDER ENCODER: " + arm.getShoulderAngle());
			// System.out.println("ELBOW ENCODER: " + arm.getElbowAngle());
			// System.out.println("END EFFECTOR X " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
			// System.out.println("END EFFECTOR Y " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
			
			smartdash.putNumber("END EFFECTOR X", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle(), arm.getWristAngle())[0]);
			smartdash.putNumber("END EFFECTOR Y", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle(), arm.getWristAngle())[1]);
			
			smartdash.putNumber("WRIST X", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
			smartdash.putNumber("WRIST Y", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
			
			smartdash.putBoolean("MOTOR LIMIS", arm.motorLimits);
			smartdash.putBoolean("isMOTOR STOPPED", arm.shouldMotorStop());
			// System.out.println(xboxController.getRightY());
			smartdash.pushDashboard(limelight, imu, driveBase, zed);
		}
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/**
	 * Switches between different driving modes
	 * DPad (POV) uses cardinal directions: 0 is up, and angles go clockwise, so 90 is right
	 */
	private void checkForModeSwitches() {
		boolean up = xboxController.getPOV() == 0;
		boolean right = xboxController.getPOV() == 90;
		boolean down = xboxController.getPOV() == 180;
		boolean left = xboxController.getPOV() == 270;

		if (up) driveBase.turnOnDefaultMode();
		if (down) driveBase.turnONPIDTuningMode();
		if (left) driveBase.turnOnDebugMode();
		if (right) driveBase.turnOnStopMode();
	}

	/**
	 * Checks for button presses and activates their functions
	 */
	private void checkForButtonPresses() {
		/*
		* PRIMARY DRIVE CONTROLLER
		*/
		boolean A = xboxController.getAButtonPressed();
		boolean B = xboxController.getBButtonPressed();
		boolean X = xboxController.getXButtonPressed();
		boolean Y = xboxController.getYButtonPressed();
		boolean RB = xboxController.getRightBumperPressed();
		boolean LB = xboxController.getLeftBumperPressed();

		if (A) {
			driveBase.clearOdom();
			System.out.println("A PRESSED");
		}
		if (B) {
			// // driveBase.printActiveMotorDebugMode();
			teleopState = TeleopStates.rampAssistedBalance;
			System.out.println("B PRESSED");
		}
		if (Y) {
			teleopState = TeleopStates.assistedAlignZED;
		}
		if (X) {
			fieldOriented = !fieldOriented;
		}
		if (RB) {
			driveBase.increaseSpeedBracket();
		}
		if (LB) {
			driveBase.decreaseSpeedBracket();
		}

		/*
		* MONKEY CONTROLLER
		*/

		boolean groundLayingDown = monkeyController.getRawButtonPressed(1);
		boolean ground = monkeyController.getRawButtonPressed(2);
		boolean substationPickup = monkeyController.getRawButtonPressed(3);
		boolean lowHeight = monkeyController.getRawButtonPressed(6);
		boolean midHeight = monkeyController.getRawButtonPressed(7);
		boolean topHeight = monkeyController.getRawButtonPressed(8);

		boolean closeClaw = monkeyController.getRawButtonPressed(4);
		boolean wristUp = monkeyController.getRawButtonPressed(5);
		boolean openClaw = monkeyController.getRawButtonPressed(9);
		boolean wristDown = monkeyController.getRawButtonPressed(10);

		boolean neutral = monkeyController.getRawButtonPressed(11);
		boolean autoClawToggle = monkeyController.getRawButtonPressed(12);
		if (autoClawToggle) autoClaw = !autoClaw;
		if (autoClaw && !limitSwitch.get()){
			autoClaw = false;
			claw.clawForward();
		}

		boolean camera0Button = monkeyController.getRawButtonPressed(13);
		boolean camera1Button = monkeyController.getRawButtonPressed(14);
		// if (camera0Button) server.setSource(camera1);

		boolean conePressed = monkeyController.getRawButtonPressed(17);
		boolean cubePressed = monkeyController.getRawButtonPressed(18);
		if (conePressed) {
			cone = true; 
			cube = false;
		}
		if (cubePressed) {
			cone = false;
			cube = true;
		}

		boolean stop = monkeyController.getRawButtonPressed(21);

		boolean speedHigh= monkeyController.getRawButtonPressed(22);
		boolean speedMid = monkeyController.getRawButtonPressed(23);
		boolean speedLow = monkeyController.getRawButtonPressed(24);

		boolean scrollUp = monkeyController.getRawButtonPressed(26);
		boolean scrollDown = monkeyController.getRawButtonPressed(27);

		/* 
		* ARM SET POSITIONS
		*/

		if (groundLayingDown) {
			arm.setEndEffector(Constants.ArmFixedPosition.PICKUP_GROUND_LAYING_DOWN);
			System.out.println("PICKUP_GROUND_LAYING_DOWN");
		}
		if (ground) {
			arm.setEndEffector(Constants.ArmFixedPosition.PICKUP_GROUND);
			System.out.println("PICKUP_GROUND");
		}
		if (substationPickup) {
			arm.setEndEffector(Constants.ArmFixedPosition.PICKUP_SUBSTATION);
			System.out.println("PICKUP_SUBSTATION");
		}
		if (lowHeight) {
			arm.setEndEffector(Constants.ArmFixedPosition.DROPOFF_LOW);
			System.out.println("DROPOFF_LOW");
		}
		if (midHeight && cone) {
			arm.setEndEffector(Constants.ArmFixedPosition.CONE_LOWER);
			System.out.println("CONE_LOWER");
		}
		if (topHeight && cone) {
			arm.setEndEffector(Constants.ArmFixedPosition.CONE_HIGHER);
			System.out.println("CONE_HIGHER");
		}
		if (midHeight && cube) {
			arm.setEndEffector(Constants.ArmFixedPosition.CUBE_LOWER);
			System.out.println("CUBE_LOWER");
		}
		if (topHeight && cube) {
			arm.setEndEffector(Constants.ArmFixedPosition.CUBE_HIGHER);
			System.out.println("CUBE_HIGHER");
		}
		if (neutral) {
			arm.setEndEffector(Constants.ArmFixedPosition.NEUTRAL);
			System.out.println("NEUTRAL");
		}

		if (stop) {
			arm.stopTrajectory();
			System.out.println("stopTrajectory");
		}

		/*
		* PNEUMATICS
		*/
		
		if (closeClaw) {
			claw.clawForward();
			System.out.println("closeClaw");
		}
		if (openClaw) {
			claw.clawBack();
			System.out.println("openClaw");
		}
		if (wristUp) {
			arm.setWrist(1);
			System.out.println("wristUp");
		}
		if (wristDown) {
			arm.setWrist(0);
			System.out.println("wristDown");
		}

		/*
		* CONTROLS
		*/
		if (speedLow) {
			arm.setSpeedMultipler(0.5);
			System.out.println("speedLow");
		}
		if (speedMid) {
			arm.setSpeedMultipler(1);
			System.out.println("speedMid");
		}
		if (speedHigh) {
			arm.setSpeedMultipler(2);
			System.out.println("speedHigh");
		}
	}

	/** 
	 * Powers motors based on the analog inputs
	 */
	private void drive() {
		// process input (determine wheelspeeds)
		// driveBase.drive(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX(), xboxController.getRightY());
		driveBase.drive(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX(), false);
	}

	// private TankDrive createTankDrive() {
	//   return new TankDrive(Constants.SPARK_MAX_ID, Constants.LEFT_VICTOR_ID,
	//                        Constants.TALON_ID, Constants.RIGHT_VICTOR_ID);
	// }

}
