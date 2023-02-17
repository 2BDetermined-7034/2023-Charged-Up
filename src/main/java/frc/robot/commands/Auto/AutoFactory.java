package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

public class AutoFactory {
    SendableChooser<Integer> startingPosition;
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
        startingPosition = new SendableChooser<>();
        startingPosition.addOption("Top", 1);
        startingPosition.addOption("Mid", 2);
        startingPosition.addOption("Low", 3);

        firstAction = new AutoAction("First", m_drive, m_arm);
        secondAction = new AutoAction("First", m_drive, m_arm);
    }
    public Command getAuto() {
        int startingPos = startingPosition.getSelected();
        switch (startingPos) {
            case 1:
                m_drive.setPosition(new Pose2d(2, 4.5, new Rotation2d()));
                break;
            case 2:
                m_drive.setPosition(new Pose2d(2, 2.8, new Rotation2d()));
                break;
            case 3:
                m_drive.setPosition(new Pose2d(2, 1, new Rotation2d()));
                break;
            default:
                break;
        }

        return firstAction.getSelected(m_visionlocker.getTeam(), startingPos);
    }


    static Command getTwoPiece(SwerveDrive m_drive) {
        PathPlannerTrajectory path = PathPlanner.loadPath("", new PathConstraints(2, 3));

        return new PathFactory(m_drive, path, true, true).getCommand();
    }
}