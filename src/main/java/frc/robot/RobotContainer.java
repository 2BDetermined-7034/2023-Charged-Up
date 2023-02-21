// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmOverride;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.clob.GravityClawToggleCommand;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.*;


public class RobotContainer {
    private final SwerveDrive m_swerveDrive = new SwerveDrive();
    private final Arm m_Arm = new Arm();
    private final XboxController m_operatorController = new XboxController(OperatorConstants.kDriverControllerPort);

    private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

    private final GravityClawSubsystem gravityClawSubsystem = new GravityClawSubsystem();

    private final GravityClawToggleCommand gravityClawToggleCommand = new GravityClawToggleCommand(gravityClawSubsystem);
    private final VisionLocking m_visionLocker = new VisionLocking();

    private final AutoBalance balance = new AutoBalance(m_swerveDrive);

    private final Intake intake = new Intake();
    private final Indexer m_indexer = new Indexer();


    public RobotContainer() {

        m_swerveDrive.setDefaultCommand(new DefaultDriveCommand(
                m_swerveDrive,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed() * (m_Arm.isArmHome() ? 1 : 0.5)),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed() * (m_Arm.isArmHome() ? 1 : 0.5)),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed() * (m_Arm.isArmHome() ? 1 : 0.5))
        ));

        intake.setDefaultCommand(new RunIntakeCommand(
                intake,
                m_indexer,
                m_driverController::getR2Axis,
                m_driverController::getL2Axis
        ));

        configureBindings();
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

    private void configureBindings() {
        m_driverController.R1().whileTrue(new ArmOverride(m_Arm, m_driverController::getLeftX, m_driverController::getRightY, m_driverController::getR2Axis));

        m_driverController.share().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));
//            m_operatorController.circle()

        //m_driverController.triangle().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::setLimeLightVision));
        //m_driverController.circle().whileTrue(new DriveToTarget(m_swerveDrive, m_visionLocker).andThen(new ChaseTagCommand(m_swerveDrive, m_visionLocker)));

        m_driverController.triangle().onTrue(gravityClawToggleCommand);



        m_driverController.square().whileTrue(balance);


        m_driverController.cross().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));

        // Gunner controls

        new POVButton(m_operatorController, 180).whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.LEFT)));
        new POVButton(m_operatorController, 0).whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.RIGHT)));
        new POVButton(m_operatorController, 90).whileTrue(m_visionLocker.runOnce(m_visionLocker::levelUp));
        new POVButton(m_operatorController, 270).whileTrue(m_visionLocker.runOnce(m_visionLocker::levelDown));


       new Trigger(m_operatorController::getLeftBumper).whileTrue(m_visionLocker.runOnce(m_visionLocker::gridLeft));
       new Trigger(m_operatorController::getRightBumper).whileTrue(m_visionLocker.runOnce(m_visionLocker::gridRight));


        new Trigger(m_operatorController::getBackButton).onTrue(gravityClawToggleCommand);
        new Trigger((() -> m_operatorController.getRightTriggerAxis() > 0.05)).onTrue(
                new ArmOverride(m_Arm, m_operatorController::getLeftX, m_operatorController::getRightY, () -> 1));

        new Trigger(m_operatorController::getYButton).onTrue(new SetArmCommand(m_Arm, intake, Units.degreesToRadians(90), Units.degreesToRadians(90))); // high
        new Trigger(m_operatorController::getXButton).onTrue(new SetArmCommand(m_Arm, intake, Units.degreesToRadians(90), Units.degreesToRadians(309))); // med
        new Trigger(m_operatorController::getBButton).onTrue(new SetArmCommand(m_Arm, intake, Units.degreesToRadians(130), Units.degreesToRadians(45))); // low



        new Trigger(m_operatorController::getBButton).onTrue(m_visionLocker.runOnce(m_visionLocker::togglePiece));


    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoFactory.getSmallSquare(m_swerveDrive);

    }


}