package frc.robot.commands.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmState;

public class ArmPathFactory {

    public static Command getIntakePath(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                    new SetArmCommand(m_arm, m_intake, new ArmState(Units.degreesToRadians(117), Units.degreesToRadians(-32), 0, 0.5)),
                    new SetArmCommand(m_arm, m_intake, new ArmState(Units.degreesToRadians(117), Units.degreesToRadians(-32), 0, 0.5)),
                    new SetArmCommand(m_arm, m_intake, new ArmState(Units.degreesToRadians(107), Units.degreesToRadians(0), 0, 0.5)),
                    new SetArmCommand(m_arm, m_intake, new ArmState(Units.degreesToRadians(65), Units.degreesToRadians(37)))
                );
    }
    public static Command getScoreHighPath(Arm m_arm, Intake m_intake){
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.tuck),
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.tuck),
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.high)

        );
    }

    public static Command getScoreMidPath(Arm m_arm, Intake m_intake) {
        return new SequentialCommandGroup(
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.tuck),
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.tuck),
                new SetArmCommand(m_arm, m_intake, Constants.ArmConstants.ArmSetPoints.mid)

        );
    }
}
