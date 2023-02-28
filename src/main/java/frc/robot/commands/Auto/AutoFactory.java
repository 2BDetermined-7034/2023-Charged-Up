package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmPathFactory;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.commands.clob.GravityClawCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GravityClawSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.preIntake;

public class AutoFactory {

    public static Command getOneConeAuto(SwerveDrive drive, Intake intake, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exit", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, intake, Constants.ArmConstants.ArmSetPoints.intake, false),
                ArmPathFactory.getScoreHighPath(arm, intake),
                new GravityClawCommand(claw, true),
                ArmPathFactory.getIntakePath(arm, intake),
                new PathFactory(drive, path, true).getCommand()
        );
    }

    public static Command getOnePieceThenLevel(SwerveDrive drive, Intake intake, GravityClawSubsystem claw, Arm arm) {
        PathPlannerTrajectory path = PathPlanner.loadPath("exitLevel", new PathConstraints(3, 3));

        return new SequentialCommandGroup(
                new GravityClawCommand(claw, false),
                new SetArmCommand(arm, intake, Constants.ArmConstants.ArmSetPoints.intake, false),
                ArmPathFactory.getScoreHighPath(arm, intake),
                new GravityClawCommand(claw, true),
                ArmPathFactory.getIntakePath(arm, intake),
                new PathFactory(drive, path, true).getCommand(),
                new AutoBalance(drive)
        );
    }

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