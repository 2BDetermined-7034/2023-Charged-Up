package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.subsystems.*;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;


public class ArmPathFactory {
    public static Command getOut(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                new GravityClawCommand(m_claw, false),
                new ParallelRaceGroup(
                            new SetArmCommand(m_arm, m_swerve, preIntake, false),
                            new RunIntakeCommand(
                                    m_swerve,
                                    m_intake,
                                    m_indexer,
                                    () -> 0,
                                    () -> -0.2,
                                    false
                            )
                        )

        );
    }

    public static Command getScoreHighPath(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer m_indexer){
        return new SequentialCommandGroup(
                getOut(m_swerve, m_claw, m_arm, m_intake, m_indexer),
                new SetArmCommand(m_arm, m_swerve, passThrough, false),
                new SetArmCommand(m_arm, m_swerve,  high, false)

        );
    }

    public static Command getScoreMidPath(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                getOut(m_swerve, m_claw, m_arm, m_intake, m_indexer),
                new SetArmCommand(m_arm, m_swerve, passThrough, false),
                new SetArmCommand(m_arm, m_swerve, midBack, false)

        );
    }

    public static Command getScoreMidFrontPath(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer indexer) {
        return new SequentialCommandGroup(
                getOut(m_swerve, m_claw, m_arm, m_intake, indexer),
                new SetArmCommand(m_arm, m_swerve, frontMid, false)

        );
    }
    public static Command getIntakePath(Arm m_arm, SwerveDrive drive, GravityClawSubsystem claw){
        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(m_arm, drive,  passThrough, false),
                new GravityClawCommand(claw, true),
                new SetArmCommand(m_arm, drive,preIntake, false),
                new SetArmCommand(m_arm, drive, intake, false),
                drive.runOnce(() -> drive.setSpeedMulti(1))
        );
    }
}
