package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SwerveDrive;

public class AutoFactory {

    public static Command getAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("path", new PathConstraints(3, 3));

        return new PathFactory(drive, path, true).getCommand();
    }

    public static Command getSpinAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Spin", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path, true).getCommand();
    }

    public static Command getSquareAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("square", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));
        return new PathFactory(drive, path, true).getCommand();
    }

    public static Command getSmallSquare(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("smallSquare", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path, true).getCommand();
    }

    public static Command getSmallSquareSpin(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("smallSquareSpin", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path, true).getCommand();
    }

}