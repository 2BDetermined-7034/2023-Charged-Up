package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.subsystems.*;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;


public class ArmPathFactory {
    public static Command getOut(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                new GravityClawCommand(m_claw, false),
                new ParallelRaceGroup(
                            new SetArmCommand(m_arm, preIntake),
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
                new SetArmCommandWithConstraints(m_arm, passThroughOut, new TrapezoidProfile.Constraints(6, 4), new TrapezoidProfile.Constraints(7, 3)),
                new SetArmCommandWithConstraints(m_arm, high, new TrapezoidProfile.Constraints(5, 2), new TrapezoidProfile.Constraints(6,3.5))
        );
    }

    public static Command getScoreMidPath(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                getOut(m_swerve, m_claw, m_arm, m_intake, m_indexer),
                new SetArmCommand(m_arm, passThroughOut),
                new SetArmCommandWithConstraints(m_arm, midBack, new TrapezoidProfile.Constraints(5, 2), new TrapezoidProfile.Constraints(6,3))
        );
    }

    public static Command getScoreShelf(SwerveDrive m_swerve, GravityClawSubsystem m_claw, Arm m_arm, Intake m_intake, Indexer indexer) {
        return new SequentialCommandGroup(
                getOut(m_swerve, m_claw, m_arm, m_intake, indexer),
                new SetArmCommandWithConstraints(m_arm, shelf, new TrapezoidProfile.Constraints(5,4), new TrapezoidProfile.Constraints(1,1)),
                new GravityClawCommand(m_claw, false)
        );
    }

    public static Command getIntakePath(Arm m_arm, GravityClawSubsystem claw){
        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommandWithConstraints(m_arm, passThrough, new TrapezoidProfile.Constraints(5, 4), new TrapezoidProfile.Constraints(4, 3)),
                new SetArmCommandWithConstraints(m_arm, preIntake, new TrapezoidProfile.Constraints(5,4), new TrapezoidProfile.Constraints(3, 1)),
                new GravityClawCommand(claw, true),
                new SetArmCommand(m_arm, intake)
        );
    }

    public static Command getIntakePathAuto(Arm m_arm, GravityClawSubsystem claw){
        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommandWithConstraints(m_arm, passThrough, new TrapezoidProfile.Constraints(5, 4), new TrapezoidProfile.Constraints(3, 1.5)),


                new SetArmCommandWithConstraints(m_arm, preIntake, new TrapezoidProfile.Constraints(5,4), new TrapezoidProfile.Constraints(3, 1)),
                new GravityClawCommand(claw, true),


                new SetArmCommand(m_arm, intake)
        );
    }

    public static Command getAutoHighPath(Arm m_arm, GravityClawSubsystem claw) {
        return new SequentialCommandGroup(
        new SetArmCommand(m_arm, new ArmState(Units.degreesToRadians(97), Units.degreesToRadians(210))),
                new SetArmCommand(m_arm, high),
                new WaitCommand(1.5)

                );
    }
}
