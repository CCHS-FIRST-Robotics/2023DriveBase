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

  Limelight limelight = new Limelight();
  BetterShuffleboard smartdash = new BetterShuffleboard();
   
  double test = 0;
  long counter = 0; // for calling functions every n loops

  public Robot() {
    // addPeriodic(() -> limelight.changePipeline(), 0.01, 0.005);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    checkForButtonPresses();

    // limelightTestDrive();
  }

  // public void limelightTestDrive() {
  //   double kP = .1;

  //   double d1 = limelight.getForwardDistance(Constants.SHORT_PIPE_NUM);
  //   double d2 = limelight.getForwardDistance(Constants.TALL_PIPE_NUM);
  //   double h = Constants.TARGETS_DISTANCE; 

  //   double l2 = d2 * d2 - d1 * d1 - h * h;
  //   double l1 = Math.sqrt(d1 * d1 - l2 * l2);

  //   double alpha = limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM);
  //   double beta = Math.atan(l2 / l1);

  //   double Fx = kP * Math.abs(l1) * Math.sin(
  //     beta + alpha
  //   );
  //   double Fy = kP * Math.abs(l1) * Math.cos(
  //     beta + alpha
  //   );

  //   driveBase.drive(Fx, Fy, 1 * limelight.getHeadingDisplacement(Constants.SHORT_PIPE_NUM), 0);
  // }

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
   * Checks for button presses and activates their functions
   */
  private void checkForButtonPresses() {
    if (xboxController.getAButtonPressed())
      limelight.updatePipeline(0);

    if (xboxController.getBButtonPressed())
      limelight.updatePipeline(1);
  }

  /** 
   * Powers motors based on the analog inputs
   */
  private void drive() {
  }

}
