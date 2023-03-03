package frc.robot.commands.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GravityClawSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;


public class ArmPathFactory {

    public static Command getScoreHighPath(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, tuck, false),
                new SetArmCommand(m_arm, tuck, false),
                new SetArmCommand(m_arm, passThrough, false),
                new SetArmCommand(m_arm, high, false)

        );
    }

    public static Command getScoreMidPath(Arm m_arm, Intake m_intake) {
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, tuck, false),
                new SetArmCommand(m_arm, tuck, false),
                new SetArmCommand(m_arm, passThrough, false),
                new SetArmCommand(m_arm, midBack, false)

        );
    }

    public static Command getScoreMidFrontPath(Arm m_arm, Intake m_intake) {
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, tuck, false),
                new SetArmCommand(m_arm, tuck, false),
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
