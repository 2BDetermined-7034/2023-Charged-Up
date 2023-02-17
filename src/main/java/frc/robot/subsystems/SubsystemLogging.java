package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public interface SubsystemLogging {

    /*Dumb Log Methods*/
    // amogus ඞඞඞඞඞඞඞඞ
    // i love coding with aaryan
    // I also love coding with aaryan based
    default void log(String ඞ, String... doub) {
        Logger.getInstance().recordOutput(ඞ, doub);
    }
     default void log(String ඞ, int doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }
     default void log(String ඞ, Pose2d... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }
     default void log(String ඞ, double... doub) {
        Logger.getInstance().recordOutput(ඞ,  (doub));
    }
     default void log(String ඞ, boolean... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }
     default void log(String ඞ, SwerveModuleState... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }
    default void log(String ඞ, long... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }

    /**
     * Where subsystems should <b><u>override</u></b> and log shit
     */
    default void updateLogging() {
        Logger.getInstance().recordOutput(String.format("%s Default", this.getClass().getName()), "amogus");
    }
}
