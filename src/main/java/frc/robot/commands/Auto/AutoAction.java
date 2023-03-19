package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;

public class AutoAction {
    ShuffleboardLayout tab;
    SendableChooser<Command> actionChooser;
    SendableChooser<Command> heightChooser;
    SwerveDrive drive;
    Arm arm;
    double timeElapsed;
    String name;
    public AutoAction(String name, SwerveDrive swerve, Arm arm){
        this.name = name;
        timeElapsed = -1;
        this.arm = arm;
        drive = swerve;

        tab = Shuffleboard.getTab("Auto").getLayout(name, BuiltInLayouts.kList);
        configureContainer();
    }

    public void configureContainer(){
        actionChooser = new SendableChooser<>();

        actionChooser.addOption("Cone L 1", getConeLeft(1));
        actionChooser.addOption("Cone R", getConeRight(1));
        actionChooser.addOption("Cube", getCube(1));
        actionChooser.addOption("Engage", getEngage(1));
        actionChooser.addOption("Leave Community", getLeaveCommunity(1));
        actionChooser.addOption("Do Nothing", new WaitCommand(0));

        heightChooser = new SendableChooser<>();
        //heightChooser.addOption("L1", new SetArmCommand(arm, low[0], low[1]));
        //heightChooser.addOption("L2", new SetArmCommand(arm, medium[0], medium[1]));
        //heightChooser.addOption("L3", new SetArmCommand(arm, high[0], high[1]));

        tab.add(name + " Action", actionChooser);
        tab.add(name + " Height", heightChooser);
    }

    public Command getConeLeft(int grid) {
        Pose2d startPose = drive.getPosition();
        Pose2d endPose = VisionLocking.getPosition(VisionLocking.Team.BLUE, grid, VisionLocking.PieceType.CONES, VisionLocking.Side.LEFT);
        PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);
        return new PathFactory(drive, path, true, false).getCommand();
    }

    public Command getConeRight(int grid) {
        Pose2d startPose = drive.getPosition();
        Pose2d endPose = VisionLocking.getPosition(VisionLocking.Team.BLUE, grid, VisionLocking.PieceType.CONES, VisionLocking.Side.RIGHT);
        PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);
        return new PathFactory(drive, path, true, false).getCommand();
    }

    public Command getCube(int grid) {
        Pose2d startPose = drive.getPosition();
        Pose2d endPose = VisionLocking.getPosition(VisionLocking.Team.BLUE, grid, VisionLocking.PieceType.CUBES, VisionLocking.Side.LEFT);
        PathPlannerTrajectory path = PathFactory.createTagPath(startPose, endPose);

        return new PathFactory(drive, path, true, false).getCommand();
    }

    public Command getEngage(int grid) {
        List<PathPoint> points = new ArrayList<>();
        points.add(PathFactory.pose2dToPathpoint(VisionLocking.exit(VisionLocking.Team.BLUE, grid)));
        points.add(new PathPoint(new Translation2d(5.45, 2.9), new Rotation2d()));
        points.add(new PathPoint(new Translation2d(3.90, 2.9), new Rotation2d()));

        PathPlannerTrajectory path = PathFactory.pathMaker(points);

        return new PathFactory(drive, path, true, false).getCommand().andThen(new AutoBalance(drive));
    }

    public Command getLeaveCommunity(int grid) {
        Pose2d startPose = drive.getPosition();
        Pose2d endPose = VisionLocking.exit(VisionLocking.Team.BLUE, grid);
        PathPlannerTrajectory path = PathFactory.createTagPath(startPose, endPose);

        return new PathFactory(drive, path, true, false).getCommand();
    }


    public Command getSelected() {
        return actionChooser.getSelected();
    }
}