package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.SetArmCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

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

        heightChooser = new SendableChooser<CommandBase>();
        heightChooser.addOption("L1", new SetArmCommand(arm, low[0], low[1]));
        heightChooser.addOption("L2", new SetArmCommand(arm, medium[0], medium[1]));
        heightChooser.addOption("L3", new SetArmCommand(arm, high[0], high[1]));
    }

    public CommandBase getSelected(Pose2d start, int grid){
        String action = actionChooser.getSelected();

        if(action.equals("CL")){
            VisionLocking.
        }





    }
}
