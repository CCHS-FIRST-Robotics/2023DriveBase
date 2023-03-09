// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.*;
import frc.robot.utils.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  Arm arm = new Arm(Constants.SHOULDER_TALON_ID, Constants.ELBOW_TALON_ID, Constants.ELBOW_FALCON_ID);
  Limelight limelight = new Limelight();
  IMU imu = new IMU();
  ZED zed = new ZED();
  BetterShuffleboard smartdash = new BetterShuffleboard();

  private Controller xboxController = new Controller();
  private MecaDrive driveBase = new MecaDrive(Constants.FL_TALON_ID, Constants.FR_TALON_ID, Constants.RL_TALON_ID, Constants.RR_TALON_ID, imu);
  
  boolean autonomousIsMoving = true;

  double test = 0;
  long counter = 0; // for calling functions every n loops

  public Robot() {
    // addPeriodic(() -> updateArmVelocities(), .001);
  }

  // public void updateArmVelocities() {
    
  // }
  long auton_counter = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);

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
    Pose2d current = driveBase.getPose();
    // start with a small displacement ( + 1)
    Pose2d target = new Pose2d(current.getX(), current.getY() + 1, current.getRotation().plus(new Rotation2d(0)));
    Autonomous.updateTrajectory(driveBase, target, new ArrayList<Translation2d>());
    driveBase.resetCurrentTrajectoryTime();
    System.out.println("zeroing counter");
    auton_counter = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     break;
		// // Default code
    // }
    // stop the robot after a while
    // if (auton_counter > 200) autonomousIsMoving = false;
		// this if statement is kind of irrelevant because we have no way to change this bool
    // because all controller input is ignored during autonomous
		if (autonomousIsMoving){
			// increase the current time, because autonomous trajectories need a time (each period takes the same time)
			driveBase.incrementCurrentTrajectoryTime(); // so add it up
			// tell the autonomous system to use it's trajectory from the drivebase to drive the robot
      Autonomous.applyChassisSpeeds(driveBase);

      // +x is forward, +y is right, +z is clockwise as viewed from above
      // driveBase.mDrive.driveCartesian(0, 0, -Math.PI/16);
      
      // +x is forward, +y is left
      // driveBase.drive(new ChassisSpeeds(.25, 0, 0));

      // it seems to be driving about 500 click/100ms too fast??
      // driveBase.frontLeftMotor.set(ControlMode.Velocity, 5500); // should be about 25%
      // driveBase.frontRightMotor.set(ControlMode.Velocity, 5500); // should be about 25%
      // driveBase.rearLeftMotor.set(ControlMode.Velocity, 5500); // should be about 25%
      // driveBase.rearRightMotor.set(ControlMode.Velocity, 5500); // should be about 25%
      driveBase.printVelocity();
      System.out.println(driveBase.getOdomHeading());
		}
    else {
      // System.out.println("Not moving");
      driveBase.drive(0, 0, 0);
    }
		auton_counter++;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    smartdash.updateControllerExponents();
    smartdash.updatePIDConstants(arm);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // switch between driving modes
    checkForModeSwitches();

    // check for button/bumper presses
    checkForButtonPresses();

    // powers motors based on the analog inputs
    // drive();
    // arm.moveArm(xboxController.getLeftX(), xboxController.getLeftY());
    // System.out.println("Alpha:" + arm.getShoulderAngle());
    // System.out.println("Beta:" + arm.getElbowAngle());
    // System.out.println("\n\n");

    if (arm.shouldMotorStop()) {
      arm.stopMotors();
      
    } else {
      arm.testMoveShoulder(xboxController.getRightX());
      arm.testMoveElbow(xboxController.getRightY());
      // arm.stopMotors();

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
    }


    if (counter % 10 == 0) {
      // System.out.println(arm.getShoulderRawAngle());

      if (arm.shouldMotorStop()) {
        System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
      }

      smartdash.putNumber("SHOULDER ENCODER", arm.getShoulderAngle());
      smartdash.putNumber("ELBOW ENCODER", arm.getElbowAngle());

      // System.out.println(arm.getShoulderFeedforward());

      // System.out.println("SHOULDER ENCODER: " + arm.getShoulderAngle());
      // System.out.println("ELBOW ENCODER: " + arm.getElbowAngle());
      // System.out.println("END EFFECTOR X " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
      // System.out.println("END EFFECTOR Y " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
      

      smartdash.putNumber("END EFFECTOR X", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
      smartdash.putNumber("END EFFECTOR Y", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
      smartdash.putBoolean("MOTOR LIMIS", arm.motorLimits);
      smartdash.putBoolean("isMOTOR STOPPED", arm.shouldMotorStop());
      // System.out.println(xboxController.getRightY());
      smartdash.pushDashboard(limelight, imu, zed);
    }
    counter++;
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
   * For the DPad, 0 is up, and angles go clockwise, so 90 is right
   */
  private void checkForModeSwitches() {
    // up
    if (xboxController.getPOV() == 0) driveBase.turnOnDefaultMode();
    // right
    if (xboxController.getPOV() == 90) driveBase.turnOnStopMode();
    // left
    if (xboxController.getPOV() == 270) driveBase.turnOnDebugMode();
    // down
    if (xboxController.getPOV() == 180) driveBase.turnONPIDTuningMode();
  }

  /**
   * Checks for button presses and activates their functions
   */
  private void checkForButtonPresses() {
    if (xboxController.getBButtonPressed()) {
      // driveBase.printActiveMotorDebugMode();
      arm.toggleManualMotorStop();
    }
    if (xboxController.getXButtonPressed() & xboxController.getYButtonPressed()) {
      arm.toggleMotorCheck();
      System.out.println("fhuhdushf");
    }
    if (xboxController.getLeftBumperPressed()) {
      // driveBase.decreaseSpeedBracket();
    }
    
  }

  /** 
   * Powers motors based on the analog inputs
   */
  private void drive() {

    // process input (determine wheelspeeds)
    // driveBase.drive(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX(), xboxController.getRightY());
    driveBase.drive(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX());
  }

  // private TankDrive createTankDrive() {
  //   return new TankDrive(Constants.SPARK_MAX_ID, Constants.LEFT_VICTOR_ID,
  //                        Constants.TALON_ID, Constants.RIGHT_VICTOR_ID);
  // }

}
