package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAction {
    ShuffleboardLayout tab;
    double timeElapsed;
    String name;
    public AutoAction(String name){
        this.name = name;
        timeElapsed = -1;

        tab = Shuffleboard.getTab("Auto").getLayout(name);
    }

    public void configureContainer(){
        SendableChooser<String> actionChooser = new SendableChooser<>();

        actionChooser.addOption("Cone L", "CL");
        actionChooser.addOption("Cone R", "CR");
        actionChooser.addOption("Cube", "CU");
        actionChooser.addOption("Engage", "EN");

        SendableChooser<CommandBase> heightChooser = new SendableChooser<CommandBase>();
        actionChooser.addOption("L1", );
    }

}
