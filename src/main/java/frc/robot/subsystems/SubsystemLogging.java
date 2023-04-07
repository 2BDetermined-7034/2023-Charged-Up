package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.Logger;

public interface SubsystemLogging {

    /*Dumb Log Methods*/
    default void log(String key, String val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
     default void log(String key, int val) {
         Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }

    /**
     * <a href="https://youtu.be/dQw4w9WgXcQ">Important Documentation Said Blicky</a>
     * @param key
     * @param val
     */
     default void log(String key, Pose2d val) {
         Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
     default void log(String key, double val) {
         Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
     default void log(String key, boolean val) {
         Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
     default void log(String key, SwerveModuleState[] val) {
         Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
    default void log(String key, long val) {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));
    }
    default void log(String key, Mechanism2d val) {

        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), key), (val));

    }


    /**
     * Where subsystems should <b><u>override</u></b> and log stuff
     * <div>
     *     <p>Call as last method in Periodic</p>
     * </div>
     *
     */
    default void updateLogging() {
        Logger.getInstance().recordOutput(String.format("%s/%s", this.getClass().getName().substring(21), "Default"), ("amogus"));
    }
}
