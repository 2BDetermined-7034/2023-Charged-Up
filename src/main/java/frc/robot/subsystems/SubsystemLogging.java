package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public interface SubsystemLogging {

    /*Dumb Log Methods*/
    default void log(String key, String... val) {
        Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
     default void log(String key, int val) {
         Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
     default void log(String key, Pose2d... val) {
         Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
     default void log(String key, double... val) {
         Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
     default void log(String key, boolean... val) {
         Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
     default void log(String key, SwerveModuleState... val) {
         Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }
    default void log(String key, long... val) {
        Logger.getInstance().recordOutput(String.format("%s %s", this.getClass().getName().replace("frc.robot.subsystems.", ""), key), (val));
    }

    /**
     * Where subsystems should <b><u>override</u></b> and log stuff
     */
    default void updateLogging() {
        Logger.getInstance().recordOutput(String.format("%s Default", this.getClass().getName()), "amogus");
    }
}
