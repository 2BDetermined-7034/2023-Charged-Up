package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmPathFactory;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.commands.clob.GravityClawToggleCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;

public class AutoFactory {

    public static Command getOneConeAuto(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exit", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, Constants.ArmConstants.ArmSetPoints.intake, false),
                ArmPathFactory.getScoreHighPath(arm, intake, indexer),
                new GravityClawToggleCommand(claw),
                new WaitCommand(0.5),
                ArmPathFactory.getIntakePath(arm, claw, intake),
                new PathFactory(drive, path, true, true).getCommand()
        );
    }

    public static Command getLevelTop(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevel", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new PathFactory(drive, path, true, true).getCommand(),
                new AutoBalance(drive)
        );
    }

    public static Command getLevelBot(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelLower", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new PathFactory(drive, path, true, true).getCommand(),
                new AutoBalance(drive)
        );
    }


    public static Command getOnePieceThenLevel(SwerveDrive drive, Intake intake, Indexer indexer, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevel", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, Constants.ArmConstants.ArmSetPoints.intake, false),
                ArmPathFactory.getScoreHighPath(arm, intake, indexer),
                new GravityClawToggleCommand(claw),
                new WaitCommand(0.5),
                ArmPathFactory.getIntakePath(arm, claw, intake),
                new PathFactory(drive, path, true, true).getCommand()
        );
    }

    public static Command getOnePieceThenLevelLower(SwerveDrive drive, Intake intake, Indexer indexer,  GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevelLower", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, Constants.ArmConstants.ArmSetPoints.intake, false),
                ArmPathFactory.getScoreHighPath(arm, intake, indexer),
                new GravityClawToggleCommand(claw),
                new WaitCommand(0.5),
                ArmPathFactory.getIntakePath(arm, claw, intake),
                new PathFactory(drive, path, true, true).getCommand()
        );
    }

    public static Command getTestAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("path", new PathConstraints(3, 3));
        return new PathFactory(drive, path,true, true).getCommand();
    }

    public static Command getSpinAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("Spin", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path, true, true).getCommand();
    }

    public static Command getSquareAuto(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("square", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));
        return new PathFactory(drive, path,true, true).getCommand();
    }

    public static Command getSmallSquare(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("smallSquare", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path,true, true).getCommand();
    }

    public static Command getSmallSquareSpin(SwerveDrive drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("smallSquareSpin", new PathConstraints(Constants.Drivebase.Auto.maxVelocity, Constants.Drivebase.Auto.maxAcceleration));

        return new PathFactory(drive, path,true, true).getCommand();
    }

}