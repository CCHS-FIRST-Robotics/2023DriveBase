// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.math.MathUtil;
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

import frc.robot.Constants.ArmFixedPosition;
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
		Testing,
		MoveArmToScore,
		WaitForArmPre,
		WaitForArm,
		OpenGrabber,
		WaitForGrabber,
		FoldArm,
		WaitForFoldedArm,
		DriveInit,
		Drive,
		DriveTaxi,
		Rotate180,
		DriveToCone,
		Idle,
		Rotate0,
		DriveToGrid,
		limeAutoAlign,
		LeaveCommunity,
		DriveBack,
		PlaceMid,
		WaitForPlaceMid,
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
	// AnalogInput autoClawInput = new AnalogInput(0);
	DigitalOutput ledOutput1 = new DigitalOutput(4);
	DigitalOutput ledOutput2 = new DigitalOutput(5);
	DigitalOutput ledOutput3 = new DigitalOutput(6);
	DigitalOutput ledOutput4 = new DigitalOutput(7);

	Grabber claw = new Grabber(Constants.CLAW_FORWARD_NUM, Constants.CLAW_BACKWARD_NUM, 
								Constants.WRIST_FORWARD_NUM, Constants.WRIST_BACKWARD_NUM);
	Arm arm = new Arm(Constants.SHOULDER_TALON_ID, Constants.SHOULDER_FALCON_ID, Constants.ELBOW_TALON_ID, Constants.ELBOW_FALCON_ID, limitSwitch, claw);

	Limelight limelight = new Limelight();
	ZED zed = new ZED();
	IMU imu = new IMU(Constants.ANALOG_GYRO_PORT);

	boolean isDone = false;

//   VideoSink server;
//   UsbCamera camera0, camera1;
	
	BetterShuffleboard smartdash = new BetterShuffleboard();

	private MecaDrive driveBase = new MecaDrive(Constants.FL_TALON_ID, Constants.FR_TALON_ID, Constants.RL_TALON_ID, Constants.RR_TALON_ID, imu, arm);
	
	boolean autonomousIsMoving = true;

	double pidTuningAlpha;
	double pidTuningBeta;

	boolean fieldOriented = true;
	boolean headingPid = true;
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
		driveBase.clearOdom();

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

		double mult = driveBase.getSpeedMultiplier();
		int output = 0;
		if (mult < .25) {
			output = 0;
		} else if (mult < .45) {
			output = 1;
		} else if (mult < .65) {
			output = 2;
		} else if (mult < .85) {
			output = 3;
		} else {
			output = 4;
		}

		if (limitSwitch.get() && autoClaw) {
			output += 5;
		}

		if (counter % 50 == 0) {
			// System.out.println("start");
			// System.out.println(output);
			// System.out.println((output & 0b0001) > 0);
			// System.out.println((output & 0b0010) > 0);
			// System.out.println((output & 0b0100) > 0);
			// System.out.println((output & 0b1000) > 0);
			// System.out.println("end");
		}

		// output will be something like 0b1011
		ledOutput1.set((output & 0b0001) > 0);
		ledOutput2.set((output & 0b0010) > 0);
		ledOutput3.set((output & 0b0100) > 0);
		ledOutput4.set((output & 0b1000) > 0);
		// SmartDashboard.putNumber("test", test);
		// limelight.test();
		// System.out.println("hello wo5rld");

		// test += 1;

		driveBase.updatePose(zed.getAprilTagPose(), zed.getPoseEstimate(), counter);

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

		// create an example trajectory		
		// Pose2d current = driveBase.getPose();
		// start with a small displacement ( + 1)
		// Pose2d target = new Pose2d(current.getX(), current.getY() + 1, current.getRotation().plus(new Rotation2d(0)));
		// Autonomous.updateTrajectory(driveBase, target, new ArrayList<Translation2d>());
		// driveBase.resetCurrentTrajectoryTime();
		
		// System.out.println("zeroing counter");
		autonCounter = 0;
		autonState = AutonStates.MoveArmToScore;
		if (Constants.MODE_TESTING) {
			autonState = AutonStates.Testing;
		}

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
			case Testing:
				// System.out.println("test");
				driveBase.rampTesting();
				break;
			case MoveArmToScore:
				System.out.println("Moving Arm");
				arm.setEndEffector(Constants.ArmFixedPosition.CONE_HIGHER_PRE_POS);
				autonState = AutonStates.WaitForArmPre;
				break;
			case WaitForArmPre:
				ArmFixedPosition armPos = Constants.GET_SECOND_PIECE ? Constants.ArmFixedPosition.CUBE_HIGHER : Constants.ArmFixedPosition.CONE_HIGHER;
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION) {
					arm.setEndEffector(armPos);
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
				autonState = AutonStates.WaitForFoldedArm;
				break;
			case WaitForFoldedArm:
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION)
					if (!isDone) {
						autonState = AutonStates.Drive;
					} else {
						autonState = AutonStates.Idle;
					}
					
				break;
			case DriveInit:
				break;
			case Drive:
				// zero all the encoders so it goes straight
				// driveBase.clearOdom();
				if (Constants.ROBOT_START_CENTER_FIELD) {
					// drives backwards to the ramp 
					// driveBase.setPosition(-3);
					if (Math.abs(imu.getPitch()) > 12) {
						System.out.println("test");
						autonState = AutonStates.BalanceAlternate;
						break;
					}
					driveBase.getOnRamp();
				} else {
					if (Constants.GET_SECOND_PIECE) {
						autonCounter = 20;
						autonState = AutonStates.Rotate180;
					} else {
						// driveBase.driveStraight(0, 0, 0, true, true);
						// drive backwards outside the community (-4 works)
						// driveBase.setPosition(-4);
						if (Math.abs(driveBase.getPoseX()) + Math.abs(driveBase.getPoseY()) < 4) {
							driveBase.driveStraight(0, .3, 0, true, true);
						} else {
							driveBase.driveStraight(0, 0, 0, true);
						}
					}
				}
				break;
			case DriveTaxi:
				if (Constants.ROBOT_START_CENTER_FIELD) {
					if (imu.getTilt() > 1) {
						autonCounter = 50;
						autonState = AutonStates.LeaveCommunity;
					}
					driveBase.drive(0, -.7, 0, false);
				} else {
					driveBase.setPosition(-4);
				}
				break;
			case Rotate180:
				if (driveBase.getPoseX() < -.5) {
					driveBase.driveStraight(0, 0, 0, true);
					driveBase.rotateFixed(180);
				} else if (Math.abs(driveBase.getPoseX()) + Math.abs(driveBase.getPoseY()) < 4) {
					driveBase.driveStraight(0, .7, 0, true, true);
				} else {
					driveBase.driveStraight(0, 0, 0, true);
				}
				
				if (Math.abs(driveBase.headingSetPoint - imu.getAngle()) < 1 && driveBase.getPoseX() < -.5) {
					arm.setEndEffector(Constants.ArmFixedPosition.PICKUP_GROUND);
					autonState = AutonStates.DriveToCone;
				}
				break;
			case DriveToCone:
				// drive if we havent gone too far
				if (Math.abs(driveBase.getPoseX()) + Math.abs(driveBase.getPoseY()) < 4.5) {
					driveBase.driveStraight(-2 * (driveBase.getPoseY()), .6, 0, true, true);
				} else {
					driveBase.driveStraight(0, 0, 0, true);
				}

				// senses a piece
				if (!limitSwitch.get()) {
					claw.clawForward();
					arm.setEndEffector(Constants.ArmFixedPosition.NEUTRAL);
					autonState = AutonStates.Rotate0;
				}
				break;
			case Idle:
				driveBase.driveStraight(0, 0, 0, false);
				break;
			case Rotate0:
				driveBase.rotateFixed(0);
				driveBase.driveStraight(0, 0, 0, true);
				
				if (Math.abs(driveBase.headingSetPoint - imu.getAngle()) < 1) {
					autonState = AutonStates.DriveToGrid;
				}
				break;
			case DriveToGrid:
				double sideOffset = 0;
				int direction = Constants.BLUE_TEAM ? -1 : 1;
				if (driveBase.getPoseX() > -1.5) {
					sideOffset = 0 * direction;
				}
				// drive if we havent gone too far
				driveBase.driveStraight(-.5 * (driveBase.getPoseY() - sideOffset), -.5, 0, true, true);
				if (driveBase.getPoseX() > 0) {
					autonState = AutonStates.Idle;
				}
				break;
			case limeAutoAlign:
                double xOffset = limelight.getX(0) - 13.5;
				if (Math.abs(xOffset) < 1 && driveBase.getPoseX() > 2) {
					autonState = AutonStates.PlaceMid;
				}
                double heading = imu.getAngle();
				if (Constants.isZero(limelight.getX(0))) {
					driveBase.drive(0, 0, 0, false);
					break;
				}

				double controlInput = -xOffset/180 * 10;
				controlInput = MathUtil.clamp(controlInput, -.4, .4);

				if (Math.abs(driveBase.headingSetPoint - heading) > 3) {
					driveBase.driveStraight(0, -.1, 0, true, true);
				} else {
					driveBase.limeAlign(controlInput + Math.signum(controlInput) * Constants.ROTATION_ADJUSTMENT, -.2);
				}
                break;
			case PlaceMid:
				driveBase.drive(0, 0, 0, false);
				arm.setEndEffector(Constants.ArmFixedPosition.CONE_HIGHER);
				autonState = AutonStates.WaitForPlaceMid;
				break;
			case WaitForPlaceMid:
				if (arm.currentMode == Arm.Mode.HOLDING_POSITION) {
					claw.clawBack();
					isDone = true;
					autonState = AutonStates.FoldArm;
				}
				break;
			case LeaveCommunity:
				autonCounter--;
				if (autonCounter == 0) {
					autonState = AutonStates.DriveBack;
				}
				driveBase.drive(0, -.3, 0, false);
				break;
			case DriveBack:
				if (Math.abs(imu.getPitch()) > 10) {
					System.out.println("test");
					autonState = AutonStates.Balance;
					break;
				} else {
					driveBase.drive(0, .9, 0, false);
				}
				break;
			case Balance:
				driveBase.rampAutoBalance();
				break;
			case BalanceAlternate:
				double velocity = imu.getTiltVelocity();
				if (Math.abs(velocity) < .1) {
					// System.out.println("VEL STOP");
					driveBase.driveStraight(0, 0, 0, false);
				} else {
					driveBase.rampAutoBalance();
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
		// driveBase.clearOdom();

		// arm.setNeutralPostion();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		double leftX = xboxController.getLeftX();
		double leftY = xboxController.getLeftY();
		double rightX = xboxController.getRightX();

		// if (!Constants.isZero(leftX) || !Constants.isZero(leftY) || !Constants.isZero(rightX)) {
		// 	if (teleopState != TeleopStates.assistedAlignLime || Math.abs(leftX) > 0.3)
		// 		teleopState = TeleopStates.manual;
		// }

		teleopState = TeleopStates.manual;

		// check for button/bumper presses
		checkForButtonPresses();
		checkForModeSwitches();

		/*
		* DRIVE CODE
		*/
		double heading, controlInput;
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
                double xOffset = limelight.getX(0) - 13.5;
                heading = imu.getAngle();
				if (Constants.isZero(xOffset)) {
					driveBase.drive(0, 0, 0, false);
					break;
				}

				controlInput = -xOffset/180 * 10;
				controlInput = MathUtil.clamp(controlInput, -.4, .4);

				if (Math.abs(driveBase.headingSetPoint - heading) > 3) {
					driveBase.driveStraight(0, leftY, 0, true);
				} else {
					driveBase.limeAlign(controlInput + Math.signum(controlInput) * Constants.ROTATION_ADJUSTMENT, leftY);
				}
                
                break;

			case assistedAlignZED:
				ZED.Position zedPos = ZED.Position.CONE;
				if (cube) zedPos = ZED.Position.CUBE;

				// handle the case where no tags are found
				if (zed.getAprilTagId() == -1) {
					driveBase.drive(0, 0, 0, false);
					break;
				}

				double[] pos = zed.getAprilTagPos(zedPos);
				double dx = pos[0];
				double dy = pos[2];
				heading = imu.getHeading();

				double controlInputX = dx * 3;
				controlInputX = MathUtil.clamp(controlInputX, -.5, .5);
				double controlInputY = (dy - .75) * 2;
				controlInputY = MathUtil.clamp(controlInputY, -.3, .3);
				// if (counter % 10 == 0) System.out.println(controlInputY + " next " + dy);
				driveBase.driveStraight(controlInputX, controlInputY, 0, true);
				break;
			case manual:
				if (headingPid)
					driveBase.driveStraight(leftX, leftY, rightX * 0.2, fieldOriented);
				else
					driveBase.drive(leftX, leftY, rightX * 0.2, fieldOriented);
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
			
			// System.out.println(zed.getAprilTagYaw());
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
			// System.out.println(autoClawInput.getValue());
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
			smartdash.pushMatchData(arm.motorLimits, arm.shouldMotorStop(), autoClaw);

			// System.out.println("heading setpoint: " + driveBase.headingSetPoint + " actual angle: " + Math.toRadians(imu.getAngle()));
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

		if (right) headingPid = true;
		if (left) headingPid = !true;
		if (down) fieldOriented = !true;
		if (up) fieldOriented = true;
		
		// if we just turned it on, we should set the set point to current heading
		if (headingPid && right) {
			driveBase.headingSetPoint = imu.getAngle();
			System.out.println("Heading PID ON");
		} if (!headingPid && left) {
			System.out.println("Heading PID OFF");
		}
	}

	/**
	 * Checks for button presses and activates their functions
	 */
	private void checkForButtonPresses() {
		/*
		* PRIMARY DRIVE CONTROLLER
		*/
		boolean down = xboxController.getAButtonPressed();
		boolean downHeld = xboxController.getAButton();
		boolean right = xboxController.getBButtonPressed();
		boolean rightHeld = xboxController.getBButton();
		boolean left = xboxController.getXButtonPressed();
		boolean leftHeld = xboxController.getXButton();
		boolean up = xboxController.getYButtonPressed();
		boolean upHeld = xboxController.getYButton();
		boolean RB = xboxController.getRightBumperPressed();
		boolean LB = xboxController.getLeftBumperPressed();
		boolean start = xboxController.getStartButtonPressed();
		boolean back = xboxController.getBackButtonPressed();

		boolean rTrigger = xboxController.getRightTriggerAxis() > .15;
		boolean lTrigger = xboxController.getLeftTriggerAxis() > .15;

		if (rTrigger || lTrigger) teleopState = TeleopStates.assistedAlignLime;

		
		if (right) driveBase.rotateFixed(-90);
		if (down) driveBase.rotateFixed(0);
		if (left) driveBase.rotateFixed(90);
		if (up) driveBase.rotateFixed(180);

		if (rightHeld && upHeld) driveBase.rotateFixed(-135);
		if (downHeld && leftHeld) driveBase.rotateFixed(45);
		if (downHeld && rightHeld) driveBase.rotateFixed(-45);
		if (leftHeld && upHeld) driveBase.rotateFixed(135);
		

		if (back) {
		}
		if (start) {
			driveBase.clearOdom();
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
		if (autoClawToggle) {
			autoClaw = !autoClaw;
			if (autoClaw)
				System.out.println("AUTO CLAW ARMED");
			else System.out.println("AUTO CLAW OFF");
		}
		if (autoClaw && !limitSwitch.get()){
			autoClaw = false;
			claw.clawForward();
		}
		
		// boolean camera0Button = monkeyController.getRawButtonPressed(13);
		// boolean camera1Button = monkeyController.getRawButtonPressed(14);
		// if (camera0Button) server.setSource(camera1);

		boolean incHeading = monkeyController.getRawButtonPressed(14);
		boolean decHeading = monkeyController.getRawButtonPressed(16);

		if (incHeading) {
			driveBase.headingSetPoint += 1.0;
			System.out.println("Setpoint: " + driveBase.headingSetPoint + " Angle: " + imu.getAngle());
		}

		if (decHeading) {
			driveBase.headingSetPoint -= 1.0;
			System.out.println("Setpoint: " + driveBase.headingSetPoint + " Angle: " + imu.getAngle());
		}

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

		boolean toggleMotorLimits = monkeyController.getRawButtonPressed(21);
		if (toggleMotorLimits) arm.toggleMotorCheck();

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
