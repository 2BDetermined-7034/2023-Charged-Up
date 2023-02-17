package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.commands.Drive.SetPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

public class AutoFactory {
    SendableChooser<Command> startingPosition;
    AutoAction firstAction;
    AutoAction secondAction;
    SwerveDrive m_drive;
    Arm m_arm;

    VisionLocking m_visionlocker;

    public AutoFactory(SwerveDrive drive, Arm arm, VisionLocking vision){
        m_drive = drive;
        m_arm = arm;
        m_visionlocker = vision;

        configureDashboard();
    }

    public void configureDashboard(){
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        startingPosition = new SendableChooser<>();
        startingPosition.addOption("1", new SetPosition(m_drive, new Pose2d(2, 4.5, new Rotation2d(180))));
        startingPosition.addOption("2",  new SetPosition(m_drive, new Pose2d(2, 2.8, new Rotation2d(180))));
        startingPosition.addOption("3",  new SetPosition(m_drive, new Pose2d(2, 1, new Rotation2d(180))));

        firstAction = new AutoAction("First", m_drive, m_arm);
        secondAction = new AutoAction("Second", m_drive, m_arm);

        tab.add("Starting", startingPosition);
    }
    public Command getAuto() {

        return startingPosition.getSelected().andThen(firstAction.getSelected());
    }


    static Command getTwoPiece(SwerveDrive m_drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("", new PathConstraints(2, 3));

        return new PathFactory(m_drive, path, true, true).getCommand();
    }
}