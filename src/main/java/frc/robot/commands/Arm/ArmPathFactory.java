package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;


public class ArmPathFactory {

    public static Command getScoreHighPath(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, high, true)

        );
    }

    public static Command getScoreMidPath(Arm m_arm, Intake m_intake) {
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, mid, true)

        );
    }

    public static Command getIntakePath(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, tuck, false),
                new SetArmCommand(m_arm, m_intake, preIntake, false),



                new SetArmCommand(m_arm, m_intake, intake, true)

        );
    }
    public static Command getTuckPathReversed(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake,intake, false),
                new SetArmCommand(m_arm, m_intake,intake, false),

                new SetArmCommand(m_arm, m_intake, tuck, true)
        );
    }


}
