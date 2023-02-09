// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Drive.DefaultDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;


public class RobotContainer {
    private final SwerveDrive m_swerveDrive = new SwerveDrive();
    private final VisionLocking m_visionLocking = new VisionLocking();
    private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

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

        m_driverController.cross().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));

        m_driverController.povLeft().whileTrue(m_visionLocking.runOnce(() -> m_visionLocking.setSide(VisionLocking.Side.LEFT)));
        m_driverController.povRight().whileTrue(m_visionLocking.runOnce(() -> m_visionLocking.setSide(VisionLocking.Side.RIGHT)));
        m_driverController.povUp().whileTrue(m_visionLocking.runOnce(m_visionLocking::levelUp));
        m_driverController.povDown().whileTrue(m_visionLocking.runOnce(m_visionLocking::levelDown));

        m_driverController.L1().whileTrue(m_visionLocking.runOnce(m_visionLocking::gridLeft));
        m_driverController.R1().whileTrue(m_visionLocking.runOnce(m_visionLocking::gridRight));

        m_driverController.square().whileTrue(m_visionLocking.runOnce(m_visionLocking::togglePiece));
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