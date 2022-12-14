// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  private TankDrive driveBase;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);


    // tank drive initialization
    driveBase = createTankDrive();    
    // mecanum drive initialization
    // driveBase = createMecanumDrive();
    
    // // set the dead zone for the controller analog sticks
    // driveBase.setDeadband(Constants.ANALOG_DEAD_ZONE);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // TODO: put the driving loop here
    // TODO: check if buttons are pressed and comment controller mapping
    if (xboxController.getRightBumperPressed()) driveBase.increaseSpeedBracket();
    if (xboxController.getLeftBumperPressed()) driveBase.decreaseSpeedBracket();

    // switch between modes (for DPad, 0 is up, and angles go clockwise, so 90 is right)
    // up
    if (xboxController.getPOV() == 0) driveBase.turnOnDefaultMode();
    // right
    if (xboxController.getPOV() == 90) driveBase.turnOnStopMode();
	// left
	if (xboxController.getPOV() == 270) driveBase.turnOnDebugMode();
    // down
    if (xboxController.getPOV() == 180) driveBase.turnONPIDTurningMode();


    // Debug controls
    if (xboxController.getRightStickButtonPressed()) driveBase.cycleWheelDebugMode();

    // actions of the four button presses
    if (xboxController.getAButtonPressed()) {
      driveBase.AButtonPressed();
    }
    if (xboxController.getBButtonPressed()) {
      driveBase.BButtonPressed();
    }
    if (xboxController.getXButtonPressed()) {
      driveBase.XButtonPressed();
    }
    if (xboxController.getYButtonPressed()) {
      driveBase.YButtonPressed();
    }

    // get analog input from xbox controller
    double leftAnalogX 	= xboxController.getLeftX();
    double leftAnalogY 	= xboxController.getLeftY();
    double rightAnalogX = xboxController.getRightX();
    double rightAnalogY = xboxController.getRightY();

    // process input (determine wheelspeeds)
    driveBase.drive(leftAnalogX, leftAnalogY, rightAnalogX, rightAnalogY);
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


  private TankDrive createTankDrive() {
    return new TankDrive(Constants.SPARK_MAX_ID, Constants.LEFT_VICTOR_ID,
                         Constants.TALON_ID, Constants.RIGHT_VICTOR_ID);
  }

  private MecaDrive createMecanumDrive() {
    return new MecaDrive(Constants.FL_TALON_PORT, Constants.FR_TALON_PORT,
                         Constants.RL_TALON_PORT, Constants.RR_TALON_PORT);
  }
}
