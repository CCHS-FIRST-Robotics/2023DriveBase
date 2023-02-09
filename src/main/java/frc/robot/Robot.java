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
import frc.robot.subsystems.*;

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


  Arm arm = new Arm(Constants.SHOULDER_TALON_ID, Constants.ELBOW_TALON_ID);
  Limelight limelight = new Limelight();
  IMU imu = new IMU();
  BetterShuffleboard smartdash = new BetterShuffleboard();
  
  double test = 0;
  long counter = 0; // for calling functions every n loops

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

    arm.testMoveShoulder(xboxController.getRightX());
    arm.testMoveElbow(xboxController.getRightY());

    // arm.moveArm(.3 * xboxController.getLeftX(), .3 * xboxController.getLeftY());

    if (counter % 10 == 0) {
      smartdash.putNumber("SHOULDER ENCODER", arm.getShoulderAngle());
      smartdash.putNumber("ELBOW ENCODER", arm.getElbowAngle());
      smartdash.putNumber("END EFFECTOR X", arm.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[0]);
      smartdash.putNumber("END EFFECTOR Y", arm.forwardKinematics(arm.getShoulderAngle(), arm.getElbowAngle())[1]);
      smartdash.pushDashboard(limelight, imu);
      System.out.println(xboxController.getRightY());
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
    if (xboxController.getAButtonPressed()) {
      driveBase.cycleMotor();
    }
    if (xboxController.getBButtonPressed()) {
      driveBase.printActiveMotorDebugMode();
    }
    if (xboxController.getLeftBumperPressed()) {
      driveBase.decreaseSpeedBracket();
    }
    if (xboxController.getRightBumperPressed()) {
      driveBase.increaseSpeedBracket();
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
