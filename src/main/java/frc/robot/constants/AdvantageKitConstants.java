package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;

import java.util.Map;

public class AdvantageKitConstants {
    private static final RobotType robot = RobotType.ROBOT_2023C;
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;

    private static final Alert invalidRobotAlert =
            new Alert("Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR);

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
                invalidRobotAlert.set(true);
                return RobotType.ROBOT_2023C;
            } else {
                return robot;
            }
        } else {
            return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
            case ROBOT_2023C:
            case ROBOT_2023P:
                return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

            case ROBOT_SIMBOT:
                return Mode.SIM;

            default:
                return Mode.REAL;
        }
    }

    public static final Map<RobotType, String> logFolders =
            Map.of(RobotType.ROBOT_2023P, "/media/sda2/");

    public enum RobotType {
        ROBOT_2023C,
        ROBOT_2023P,
        ROBOT_SIMBOT
    }

    public enum Mode {
        REAL,
        REPLAY,
        SIM
    }

}
