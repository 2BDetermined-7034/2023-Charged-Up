package frc.robot.commands.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Drive.PathFactory;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

import java.util.HashMap;

public class AutoFactory {
    static HashMap<String, Command> eventMap = new HashMap<>();
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
        Command first = firstAction.getSelected(m_visionlocker.getTeam(), startingPosition.getSelected());

        return first;
    }
}