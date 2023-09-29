// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmOverride;
import frc.robot.commands.Arm.ArmPathFactory;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.Drive.*;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.clob.GravityClawToggleCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.util.ArmState;


public class RobotContainer implements SubsystemLogging {
    private final SendableChooser<Command> autoMode = new SendableChooser<Command>();
    private final SwerveDrive m_swerveDrive = new SwerveDrive();
    public final Arm m_Arm = new Arm();
    private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);
    private final CommandPS4Controller m_driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
    private final GravityClawSubsystem gravityClawSubsystem = new GravityClawSubsystem();
    private final VisionLocking m_visionLocker = new VisionLocking();
    private final Intake intake = new Intake();
    private final Indexer m_indexer = new Indexer();

    public RobotContainer() {

        autoMode.addOption("One Piece",  AutoFactory.getOnePieceAuto(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));
        autoMode.addOption("One Piece Exit",  AutoFactory.getOnePieceExit(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));
        autoMode.addOption("One Piece Level open",  AutoFactory.getOnePieceThenLevelOpen(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));
        autoMode.addOption("One Piece Level mid",  AutoFactory.getOnePieceThenLevelMid(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));
        autoMode.addOption("One Piece Level cable",  AutoFactory.getOnePieceThenLevelCable(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));
        autoMode.addOption("Level Open Exit Top", AutoFactory.getLevelOpen(m_swerveDrive));
        autoMode.addOption("Exit Bot", AutoFactory.getLevelBotCable(m_swerveDrive));
        autoMode.addOption("Two Piece", AutoFactory.getTwoPiece(m_swerveDrive, intake, m_indexer, gravityClawSubsystem, m_Arm));


        autoMode.addOption("Test auto",  AutoFactory.getTestAuto(m_swerveDrive));
        autoMode.setDefaultOption("Do Nothing",  new WaitCommand(10));

        m_Arm.setGoalState(m_Arm.getCurrentState());

        m_driverController.share().whileTrue(m_swerveDrive.runOnce(m_swerveDrive::zeroGyroscope));
        m_swerveDrive.setDefaultCommand(new DefaultDriveCommand(
                m_swerveDrive,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed()),
                m_Arm
        ));

        configureBindings();

        SmartDashboard.putData("Auto",autoMode);
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
        value = deadband(value, 0.12);
        // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    private void configureBindings() {
        m_driverController.triangle().onTrue(new HeadingDriveCommand(
                m_swerveDrive,
                0,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed()))
        );

        m_driverController.circle().onTrue(new HeadingDriveCommand(
                m_swerveDrive,
                270,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed()))
        );

        m_driverController.cross().onTrue(new HeadingDriveCommand(
                m_swerveDrive,
                180,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed()))
        );
        m_driverController.square().onTrue(new HeadingDriveCommand(
                m_swerveDrive,
                90,
                () -> -square(modifyAxis(m_driverController.getLeftY()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getLeftX()) * m_swerveDrive.getMaxSpeed()),
                () -> -square(modifyAxis(m_driverController.getRightX()) * m_swerveDrive.getMaxSpeed()))
        );

        new Trigger(() -> m_driverController.getR2Axis() > 0.25).whileTrue(new RunIntakeCommand(
                m_swerveDrive,
                intake,
                m_indexer,
                () -> m_visionLocker.getPieceType().equals(VisionLocking.PieceType.CONES) ? 0.75 : 0.1,
                () -> 0.6,
                true
        ));

        new Trigger(() -> m_driverController.getL2Axis() > 0.25).whileTrue(new RunIntakeCommand(
                m_swerveDrive,
                intake,
                m_indexer,
                () -> 0,
                () -> -0.8,
                false
        ));

        m_driverController.R1().whileTrue(
                    new RunIntakeCommand(
                    m_swerveDrive,
                    intake,
                    m_indexer,
                    () -> 0,
                    () -> 0.7,
                    false)
        );
/*
The below code creates a time deadband for running the intake Solenoid, has a bug which can make the intake not come back up
 */
//        new Trigger(() -> m_driverController.getR2Axis() < 0.25).whileTrue(
//                new SequentialCommandGroup(
//                        new WaitCommand(0.75),
//                        intake.runOnce(() -> intake.setSolenoid(true))
//                )
//        );

        m_driverController.L1().onTrue(m_visionLocker.runOnce(m_visionLocker::togglePiece));

        m_driverController.povUp().whileTrue(new AutoBalance(m_swerveDrive));
        m_driverController.povDown().whileTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(1)));
        m_driverController.touchpad().whileTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(0.2)));
        
        /*
        m_driverController.touchpad().whileTrue(
                new ChaseTagCommand(m_swerveDrive, m_visionLocker).andThen(
                        new GravityClawCommand(gravityClawSubsystem, false).andThen(
                        m_visionLocker.getArmCommand(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer))
                )
        );

         */

        // Gunner controls
        new POVButton(m_operatorController, 270).whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.LEFT)));
        new POVButton(m_operatorController, 90).whileTrue(m_visionLocker.runOnce(() -> m_visionLocker.setSide(VisionLocking.Side.RIGHT)));
        new POVButton(m_operatorController, 0).whileTrue(m_visionLocker.runOnce(m_visionLocker::levelUp));
        new POVButton(m_operatorController, 180).whileTrue(m_visionLocker.runOnce(m_visionLocker::levelDown));


        new Trigger(m_operatorController::getRightStickButton).whileTrue(m_Arm.runOnce(
                () -> m_Arm.setGoalState(m_Arm.getCurrentState().clear())
        ));
        
       new Trigger(m_operatorController::getLeftBumper).whileTrue(m_visionLocker.runOnce(m_visionLocker::gridLeft));
       new Trigger(m_operatorController::getRightBumper).whileTrue(m_visionLocker.runOnce(m_visionLocker::gridRight));

       new Trigger(m_operatorController::getBackButton).onTrue(new GravityClawToggleCommand(gravityClawSubsystem));
        new Trigger(m_operatorController::getBackButton).onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(1)));

        new Trigger(m_operatorController::getAButton).onTrue(ArmPathFactory.getIntakePath(m_Arm, gravityClawSubsystem)); // high// med
        //new Trigger(m_operatorController::getRightBumper).onTrue(ArmPathFactory.getIntakePathNoLimit(m_Arm, gravityClawSubsystem)); // high// med

        new Trigger(m_operatorController::getBButton).onTrue(ArmPathFactory.getScoreMidPath(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer)); // low
        new Trigger(m_operatorController::getBButton).onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(0.3))); // low

        new Trigger(m_operatorController::getYButton).onTrue(ArmPathFactory.getScoreHighPath(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer)); // low
        new Trigger(m_operatorController::getYButton).onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(0.3))); // low

        new Trigger(m_operatorController::getStartButton).onTrue(ArmPathFactory.getScoreShelf(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer)); // low
        new Trigger(m_operatorController::getStartButton).onTrue(m_swerveDrive.runOnce(() -> m_swerveDrive.setSpeedMulti(0.3))); // low




        //new Trigger(m_operatorController::getXButton).onTrue(ArmPathFactory.getScoreShelf(m_swerveDrive, gravityClawSubsystem, m_Arm, intake, m_indexer));

        new Trigger((() -> Math.abs(m_operatorController.getLeftTriggerAxis()) > 0.05)).onTrue(
                new SetArmCommand(m_Arm, Constants.ArmConstants.ArmSetPoints.tuck));
        new Trigger((() -> Math.abs(m_operatorController.getRightTriggerAxis()) > 0.05)).onTrue(
                new ArmOverride(m_Arm, m_swerveDrive, m_operatorController::getLeftY, m_operatorController::getRightY, m_operatorController::getRightTriggerAxis));
    }

    public Command getAutonomousCommand() {
        // Send auto dropdown 
        return autoMode.getSelected();

    }


}
