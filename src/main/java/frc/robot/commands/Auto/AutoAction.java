package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.constants.Constants.ArmConstants.ArmSetPoints.*;

public class AutoAction {
    ShuffleboardLayout tab;
    SendableChooser<String> actionChooser;
    SendableChooser<CommandBase> heightChooser;
    SwerveDrive drive;
    Arm arm;
    double timeElapsed;
    String name;
    public AutoAction(String name, SwerveDrive swerve, Arm arm){
        this.name = name;
        timeElapsed = -1;
        this.arm = arm;
        drive = swerve;

        tab = Shuffleboard.getTab("Auto").getLayout(name);
        configureContainer();
    }

    public void configureContainer(){
        actionChooser = new SendableChooser<>();

        actionChooser.addOption("Cone L", "CL");
        actionChooser.addOption("Cone R", "CR");
        actionChooser.addOption("Cube", "CU");
        actionChooser.addOption("Engage", "EN");
        actionChooser.addOption("Leave Community", "LC");
        actionChooser.addOption("Do Nothing", "NA");

        heightChooser = new SendableChooser<CommandBase>();
        heightChooser.addOption("L1", new SetArmCommand(arm, low[0], low[1]));
        heightChooser.addOption("L2", new SetArmCommand(arm, medium[0], medium[1]));
        heightChooser.addOption("L3", new SetArmCommand(arm, high[0], high[1]));
    }

    public Command getSelected(VisionLocking.Team team, int grid){
        String action = actionChooser.getSelected();
        Pose2d startPose = drive.getPosition();
        if(action.equals("CL")){
            Pose2d endPose = VisionLocking.getPosition(team, grid, VisionLocking.PieceType.CONES, VisionLocking.Side.LEFT);
            PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);

            return new PathFactory(drive, path, false, true).getCommand().andThen(heightChooser.getSelected());

        } else if (action.equals("CR")){
            Pose2d endPose = VisionLocking.getPosition(team, grid, VisionLocking.PieceType.CONES, VisionLocking.Side.RIGHT);
            PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);

            return new PathFactory(drive, path, false, true).getCommand().andThen(heightChooser.getSelected());

        } else if (action.equals("CU")){
            Pose2d endPose = VisionLocking.getPosition(team, grid, VisionLocking.PieceType.CUBES, VisionLocking.Side.LEFT);
            PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);

            return new PathFactory(drive, path, false, true).getCommand().andThen(heightChooser.getSelected());
        }  else if (action.equals("EN")) {
            List<PathPoint> points = new ArrayList<>();
            points.add(PathFactory.pose2dToPathpoint(VisionLocking.exit(team, grid)));
            points.add(new PathPoint(new Translation2d(5.45, 2.9), new Rotation2d()));
            points.add(new PathPoint(new Translation2d(3.90, 2.9), new Rotation2d()));

            PathPlannerTrajectory path = PathFactory.pathMaker(points);

            return new PathFactory(drive, path, false, true).getCommand().andThen(new AutoBalance(drive));
        } else if (action.equals("LC")) {
            Pose2d endPose = VisionLocking.exit(team, grid);
            PathPlannerTrajectory path = PathFactory.createTagPath(startPose,endPose);

            return new PathFactory(drive, path, false, true).getCommand();
        }
        return new WaitCommand(0);
    }
}
