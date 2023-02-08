// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot {
  private RobotContainer robotContainer;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;


  public Robot() {
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */public void robotInit() {



    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  public void robotPeriodic() {
  }

  /** This function is called once each time the robot enters Disabled mode. */
  public void disabledInit() {}

  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */

  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */

  public void autonomousPeriodic() {}

  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  public void teleopPeriodic() {
    NetworkTableInstance.getDefault().flush();
  }
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  public void simulationPeriodic() {}
}
