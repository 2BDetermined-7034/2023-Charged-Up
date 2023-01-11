// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Drive.DefaultDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive;


public class RobotContainer {
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

    m_swerveDrive.setDefaultCommand(new DefaultDriveCommand(
            m_swerveDrive,
            () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
            () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
            () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed())
    ));
    configureBindings();
  }

  private void configureBindings() {

    m_driverController.b().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoFactory.getSquareAuto(m_swerveDrive);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.075);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }
}