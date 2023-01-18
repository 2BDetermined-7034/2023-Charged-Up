package frc.robot.commands.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

public class PathFactory {
    SwerveDrive m_swerveDrive;
    private final Command followTrajectoryCommand;

    public PathFactory(SwerveDrive drive, PathPlannerTrajectory path, boolean isFirstPath) {

        m_swerveDrive = drive;

        followTrajectoryCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        m_swerveDrive.setPosition(path.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        path,
                        m_swerveDrive::getPosition, // Pose supplier
                        m_swerveDrive.getKinematics(), // SwerveDriveKinematics
                        new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(Constants.Drivebase.Auto.kP, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        m_swerveDrive::setModuleStates, // Module states consumer
                        m_swerveDrive // Requires this drive subsystem
                )
        );
    }

    public Command getCommand() {
        return followTrajectoryCommand;
    }

    public static PathPlannerTrajectory pathMaker(List<PathPoint> points){
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration),
            points
        );
        return trajectory;
    }

}