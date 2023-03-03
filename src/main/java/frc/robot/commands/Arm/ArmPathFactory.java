package frc.robot.commands.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.RunIndexerCommand;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GravityClawSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;


public class ArmPathFactory {
    public static Command getOut(Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                            new SetArmCommand(m_arm, tuck, false),
                            new RunIntakeCommand(
                                    m_intake,
                                    m_indexer,
                                    () -> 0,
                                    () -> -0.1,
                                    false
                            )
                        )
        );
    }

    public static Command getScoreHighPath(Arm m_arm, Intake m_intake, Indexer m_indexer){
        return new SequentialCommandGroup(
                getOut(m_arm, m_intake, m_indexer),
                new SetArmCommand(m_arm, passThrough, false),
                new SetArmCommand(m_arm, high, false)

        );
    }

    public static Command getScoreMidPath(Arm m_arm, Intake m_intake, Indexer m_indexer) {
        return new SequentialCommandGroup(
                getOut(m_arm, m_intake, m_indexer),
                new SetArmCommand(m_arm, passThrough, false),
                new SetArmCommand(m_arm, midBack, false)

        );
    }

    public static Command getScoreMidFrontPath(Arm m_arm, Intake m_intake, Indexer indexer) {
        return new SequentialCommandGroup(
                getOut(m_arm, m_intake, indexer),
                new SetArmCommand(m_arm, frontMid, false)

        );
    }
    public static Command getIntakePath(Arm m_arm, GravityClawSubsystem claw, Intake m_intake){
        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(m_arm, passThrough, false),
                new SetArmCommand(m_arm, tuck, false),
                new GravityClawCommand(claw, true),
                new SetArmCommand(m_arm, preIntake, false),
                new SetArmCommand(m_arm, intake, false)

        );
    }
}
