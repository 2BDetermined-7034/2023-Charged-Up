// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.Arm.ArmOverride;
import frc.robot.commands.Arm.SetArmCommand;

import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.commands.clob.GravityClawToggleCommand;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.GravityClawSubsystem;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;


public class RobotContainer {
    private final SwerveDrive m_swerveDrive = new SwerveDrive();
    private final Arm m_Arm = new Arm();
    private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    private final CommandPS4Controller m_operatorController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    private final GravityClawSubsystem gravityClawSubsystem = new GravityClawSubsystem();

    private final GravityClawCommand gravityClawCommandTrue = new GravityClawCommand(gravityClawSubsystem, true);
    private final GravityClawCommand gravityClawCommandFalse = new GravityClawCommand(gravityClawSubsystem, false);
    private final GravityClawToggleCommand gravityClawToggleCommand = new GravityClawToggleCommand(gravityClawSubsystem);
    private final VisionLocking m_visionLocker = new VisionLocking();

    private final AutoBalance balance = new AutoBalance(m_swerveDrive);



    public RobotContainer() {

        m_swerveDrive.setDefaultCommand(new DefaultDriveCommand(
                m_swerveDrive,
                () -> -square(modifyAxis(m_driverController.getLeftY() ) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed())
        ));
        configureBindings();
    }

    private void configureBindings() {
        m_driverController.R1().whileTrue(new ArmOverride(m_Arm, () -> m_driverController.getLeftX(), () -> m_driverController.getRightY(), () -> m_driverController.getR2Axis()));

        m_driverController.share().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));
//            m_operatorController.circle()

        //m_driverController.triangle().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::setLimeLightVision));
        //m_driverController.circle().whileTrue(new DriveToTarget(m_swerveDrive, m_visionLocker).andThen(new ChaseTagCommand(m_swerveDrive, m_visionLocker)));

        m_driverController.triangle().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(90)));
        m_driverController.square().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(309)));
        m_driverController.circle().onTrue(new SetArmCommand(m_Arm,Units.degreesToRadians(130),  Units.degreesToRadians(45)));
        m_driverController.cross().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(270)));

        m_operatorController.circle().onTrue(gravityClawCommandTrue);
        m_operatorController.square().onTrue(gravityClawCommandFalse);
        m_operatorController.triangle().onTrue(gravityClawToggleCommand);


        m_driverController.triangle().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(90)));
        m_driverController.square().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(309)));
        m_driverController.circle().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(130), Units.degreesToRadians(45)));
        m_driverController.cross().onTrue(new SetArmCommand(m_Arm, Units.degreesToRadians(90), Units.degreesToRadians(270)));

        // Gunner controls

        m_operatorController.cross().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));

        m_operatorController.povLeft().whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.LEFT)));
        m_operatorController.povRight().whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.RIGHT)));
        m_operatorController.povUp().whileTrue(m_visionLocker.runOnce(m_visionLocker::levelUp));
        m_operatorController.povDown().whileTrue(m_visionLocker.runOnce(m_visionLocker::levelDown));

        m_operatorController.L1().whileTrue(m_visionLocker.runOnce(m_visionLocker::gridLeft));
        m_operatorController.R1().whileTrue(m_visionLocker.runOnce(m_visionLocker::gridRight));

        //m_operatorController.square().whileTrue(m_visionLocker.runOnce(m_visionLocker::togglePiece));

        m_operatorController.square().whileTrue(balance);

    }


    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoFactory.getSmallSquare(m_swerveDrive);

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
        value = deadband(value, 0.031);
        // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }


}