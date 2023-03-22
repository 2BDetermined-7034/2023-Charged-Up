package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Arm.ArmPathFactory;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.commands.clob.GravityClawToggleCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;

public class AutoFactory {

    public static Command getOnePieceAuto(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, Constants.ArmConstants.ArmSetPoints.intake),
                ArmPathFactory.getScoreHighPath(drive, claw, arm, intake, indexer),
                new GravityClawToggleCommand(claw),
                new WaitCommand(0.5)
        );
    }

    public static Command getLevelOpen(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevel", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new PathFactory(drive, path, true, true).getCommand(),
                new AutoBalance(drive)
        );
    }
    public static Command getOnePieceExit(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exit", new PathConstraints(2, 3));

        return new SequentialCommandGroup(
                drive.runOnce(drive.getLimeLight()::setModeDriver),
                getOnePieceAuto(drive, intake, indexer, claw, arm),
                new PathFactory(drive, path, true, true).getCommand(),
                drive.runOnce(drive.getLimeLight()::setModeVision)
        );
    }

    public static Command getOnePieceThenLevelMid(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelMid", new PathConstraints(2, 3));

        return new SequentialCommandGroup(
                drive.runOnce(drive.getLimeLight()::setModeDriver),
                getOnePieceAuto(drive, intake, indexer, claw, arm),
                new ParallelCommandGroup(
                        ArmPathFactory.getIntakePath(arm, claw),
                        new PathFactory(drive, path, true, true).getCommand()
                ),
                new AutoBalance(drive),
                drive.runOnce(drive.getLimeLight()::setModeVision)
        );
    }


    public static Command getLevelBotCable(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelCable", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new PathFactory(drive, path, true, true).getCommand(),
                new AutoBalance(drive)
        );
    }


    public static Command getOnePieceThenLevelOpen(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelOpen", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                getOnePieceAuto(drive, intake, indexer, claw, arm),
                new ParallelCommandGroup(
                        ArmPathFactory.getIntakePath(arm, claw),
                        new PathFactory(drive, path, true, true).getCommand()
                ),
                new AutoBalance(drive)

        );
    }

    public static Command getOnePieceThenLevelCable(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelCable", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                getOnePieceAuto(drive, intake, indexer, claw, arm),
                new ParallelCommandGroup(
                        ArmPathFactory.getIntakePath(arm, claw),
                        new PathFactory(drive, path, true, true).getCommand()
                ),
                new AutoBalance(drive)
        );
    }

    public static Command getTestAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("path", new PathConstraints(3, 3));
        return new PathFactory(drive, path,true, true).getCommand();
    }


}