// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.cameraserver.CameraServer;
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

  private Controller xboxController = new Controller();
  private MecaDrive driveBase;

  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_ID);

  Arm arm = new Arm(Constants.SHOULDER_TALON_ID, Constants.ELBOW_TALON_ID, Constants.ELBOW_FALCON_ID, limitSwitch);
  Limelight limelight = new Limelight();
  IMU imu = new IMU();
  ZED zed = new ZED();
  BetterShuffleboard smartdash = new BetterShuffleboard();

  boolean trajStarted = false;
  double pidTuningAlpha;
  double pidTuningBeta;
  double[][] trajectory;
  int trajectoryCounter;
  Grabber claw = new Grabber(Constants.CLAW_FORWARD_NUM, Constants.CLAW_BACKWARD_NUM, 
                             Constants.WRIST_FORWARD_NUM, Constants.WRIST_BACKWARD_NUM);
  
  double test = 0;
  long counter = 0; // for calling functions every n loops

  public Robot() {
    // addPeriodic(() -> updateArmVelocities(), .001);
  }

  // public void updateArmVelocities() {
    
  // }

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

    // mecanum drive initialization
    driveBase = createMecanumDrive();
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

    
    // SmartDashboard.putNumber("test", test);
    // limelight.test();
    // System.out.println("hello wo5rld");

    // test += 1;

    arm.updatePrevAngles();
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
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
      // // arm.stopMotors();
      // arm.setShoulder(0);
      // arm.setElbow(pidTuningAngle);
      // arm.setEndEffector(1, 1, arm.getWristAngle());

      if (trajStarted) {
        if (trajectoryCounter >= trajectory.length) {
          trajectoryCounter = trajectory.length - 1;
        } 
        double[] angles = trajectory[trajectoryCounter];
        pidTuningAlpha = angles[0];
        pidTuningBeta = angles[1];
        trajectoryCounter++;

        arm.setElbow(pidTuningBeta);
        arm.setShoulder(pidTuningAlpha);
      }


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
      // Kinematics.positionInverseKinematics(1, 1, arm.getWristAngle());

      if (arm.shouldMotorStop()) {
        System.out.println("HOLY SHIT EVERYTHING IS EXPLODING");
      }

      // System.out.println(limitSwitch.get());
      // System.out.println("test working");

      smartdash.putNumber("SHOULDER ENCODER", arm.getShoulderAngle());
      smartdash.putNumber("ELBOW ENCODER", arm.getElbowAngle());

      smartdash.putNumber("DESIRED ALPHA", pidTuningAlpha);
      smartdash.putNumber("DESIRED BETA", pidTuningBeta);

      smartdash.putNumber("Count:", trajectoryCounter);

      // System.out.println(arm.getShoulderFeedforward());

      // System.out.println("SHOULDER ENCODER: " + arm.getShoulderAngle());
      // System.out.println("ELBOW ENCODER: " + arm.getElbowAngle());
      // System.out.println("END EFFECTOR X " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
      // System.out.println("END EFFECTOR Y " + Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
      
      smartdash.putNumber("END EFFECTOR X", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle(), arm.getWristAngle())[0]);
      smartdash.putNumber("END EFFECTOR Y", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle(), arm.getWristAngle())[1]);
      
      // smartdash.putNumber("WRIST X", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
      // smartdash.putNumber("WRIST Y", Kinematics.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
      
      smartdash.putBoolean("MOTOR LIMIS", arm.motorLimits);
      smartdash.putBoolean("isMOTOR STOPPED", arm.shouldMotorStop());
      // System.out.println(xboxController.getRightY());
      smartdash.pushDashboard(limelight, imu, zed);
    }
    counter++;
  }

  public void limelightTestDrive() {
    double kP = .1;

    double d1 = limelight.getForwardDistance(Constants.SHORT_PIPE_NUM);
    double d2 = limelight.getForwardDistance(Constants.TALL_PIPE_NUM);
    double h = Constants.TARGETS_DISTANCE; 

    double l2 = d2 * d2 - d1 * d1 - h * h;
    double l1 = Math.sqrt(d1 * d1 - l2 * l2);

    double alpha = limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM);
    double beta = Math.atan(l2 / l1);

    double Fx = kP * Math.abs(l1) * Math.sin(
      beta + alpha
    );
    double Fy = kP * Math.abs(l1) * Math.cos(
      beta + alpha
    );

    driveBase.drive(Fx, Fy, 1 * limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM), 0);
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
    if (xboxController.getAButtonPressed()) {
      // driveBase.cycleMotor();
      System.out.println("THING");
      // claw.wristForward();
      claw.clawForward();
    }
    if (xboxController.getYButtonPressed()) {
      // claw.wristBack();
      claw.clawBack();
    }
    if (xboxController.getBButtonPressed()) {
      // // driveBase.printActiveMotorDebugMode();
      arm.toggleManualMotorStop();
    }
    if (xboxController.getXButtonPressed() & xboxController.getYButtonPressed()) {
      arm.toggleMotorCheck();
      System.out.println("fhuhdushf");
      claw.wristBack();
    }
    if (xboxController.getAButtonPressed()) {
      trajStarted = true;
      // pidTuningAngle = 10;
      trajectory = Kinematics.degrees(arm.getTrajectory(1, 1.5));

      System.out.println(trajectory.length);

      for (int i=0; i<trajectory.length; i++) {
        double[] angles = trajectory[i];
        System.out.println("Angles: " + angles[0] + " next " + angles[1]);
      }
      double[] angles = trajectory[trajectory.length - 1];
      System.out.println("LAST: " + angles[0] + " next " + angles[1]);
    }
    if (xboxController.getRightBumperPressed()) {
      // driveBase.increaseSpeedBracket();
      claw.wristForward();
    }
  }

  /** 
   * Powers motors based on the analog inputs
   */
  private void drive() {


    // process input (determine wheelspeeds)
    driveBase.drive(xboxController.getLeftX(), xboxController.getLeftY(), xboxController.getRightX(), xboxController.getRightY());
    // System.out.println(xboxController.getLeftY());
  }

  // private TankDrive createTankDrive() {
  //   return new TankDrive(Constants.SPARK_MAX_ID, Constants.LEFT_VICTOR_ID,
  //                        Constants.TALON_ID, Constants.RIGHT_VICTOR_ID);
  // }

  private MecaDrive createMecanumDrive() {
    return new MecaDrive(Constants.FL_TALON_ID, Constants.FR_TALON_ID,
                         Constants.RL_TALON_ID, Constants.RR_TALON_ID);
  }

}
