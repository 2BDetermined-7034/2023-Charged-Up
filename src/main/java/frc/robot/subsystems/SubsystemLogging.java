package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public interface SubsystemLogging {

    /*Dumb Log Methods*/
    // amogus amognusamognusamognusamognusamognusamognusamognusamognus
    // i love coding with aaryan
    // I also love coding with aaryan based
    default void log(String amognus, String... doub) {
        Logger.getInstance().recordOutput(amognus, doub);
    }
     default void log(String amognus, int doub) {
        Logger.getInstance().recordOutput(amognus, (doub));
    }
     default void log(String amognus, Pose2d... doub) {
        Logger.getInstance().recordOutput(amognus, (doub));
    }
     default void log(String amognus, double... doub) {
        Logger.getInstance().recordOutput(amognus,  (doub));
    }
     default void log(String amognus, boolean... doub) {
        Logger.getInstance().recordOutput(amognus, (doub));
    }
     default void log(String amognus, SwerveModuleState... doub) {
        Logger.getInstance().recordOutput(amognus, (doub));
    }
    default void log(String amognus, long... doub) {
        Logger.getInstance().recordOutput(amognus, (doub));
    }

    /**
     * Where subsystems should <b><u>override</u></b> and log shit
     */
    default void updateLogging() {
        Logger.getInstance().recordOutput(String.format("%s Default", this.getClass().getName()), "amogus");
    }
}
