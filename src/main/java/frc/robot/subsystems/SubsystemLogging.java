package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

import java.util.Arrays;

public interface SubsystemLogging {

    /*Dumb Log Methods*/
    // amogus ඞඞඞඞඞඞඞඞ
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
        Logger.getInstance().recordOutput(ඞ, (doub));
    }

    default void log(String ඞ, boolean... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }

    default void log(String ඞ, SwerveModuleState... doub) {
        Logger.getInstance().recordOutput(ඞ, (doub));
    }

    default void log(String ඞ, Long... doub) {
        Logger.getInstance().recordOutput(ඞ, Arrays.toString(doub));
    }

    default void pdpLog() {
        LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
    }
    /**
     * banana feet minion babaaaa
     */
    /**
     * Where subsystems should override and log shit
     */
    default void configureLogging() {
    }
}
