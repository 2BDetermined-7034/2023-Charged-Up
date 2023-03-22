package frc.robot.commands.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.YAGSLswerve;
import swervelib.math.SwerveModuleState2;

import java.util.HashMap;
import java.util.List;

public class PathFactory {
    private final Command followTrajectoryCommand;
    YAGSLswerve m_swerveDrive;

    public PathFactory(YAGSLswerve drive, PathPlannerTrajectory path, boolean useAlliance, boolean isFirstPath, HashMap<String, Command> eventMap) {

        m_swerveDrive = drive;
        m_swerveDrive.addTrajectory(path);


        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_swerveDrive::getPosition, // Pose supplier
                m_swerveDrive::setPosition,
                m_swerveDrive.getKinematics(), // SwerveDriveKinematics
                new PIDConstants(Constants.Drivebase.Auto.kP, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDConstants(Constants.Drivebase.Auto.kP, 0, 0), // Y controller (usually the same values as X controller)
                m_swerveDrive::setModuleStates, // Module states consumer
                eventMap,
                useAlliance, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                m_swerveDrive // Module states consumer // The drive subsystem. Used to properly set the requirements of path following commands
        );
        Command fullAuto = autoBuilder.fullAuto(path);

        followTrajectoryCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        m_swerveDrive.setPosition(path.getInitialHolonomicPose());
                    }
                }),
                fullAuto
        );
    }

    public PathFactory(SwerveDrive drive, PathPlannerTrajectory path, boolean useAlliance, boolean isFirstPath){
        this(drive, path, useAlliance, isFirstPath, new HashMap<>());
    }

    public static PathPlannerTrajectory pathMaker(List<PathPoint> points) {
        return PathPlanner.generatePath(
                new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration),
                points
        );
    }

    public static PathPlannerTrajectory createTagPath(Pose2d startPos, Pose2d endPos) {

        return PathPlanner.generatePath(
                new PathConstraints(1, 1),
                pose2dToPathpoint(startPos),
                pose2dToPathpoint(endPos)
        );

    }

    public static PathPoint pose2dToPathpoint(Pose2d point) {
        return new PathPoint(point.getTranslation(), point.getRotation());
    }

    public Command getCommand() {
        return followTrajectoryCommand.andThen(m_swerveDrive::stop);
    }


}