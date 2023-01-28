// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.util.ArmState;

public class SetArmCommand extends CommandBase {

    private final Arm arm;


    /** Creates a new ArmCommand. */

    public SetArmCommand(Arm arm, double x, double y ) {
        this.arm = arm;
        ArmState goalState = Arm.inverseKinematics(x,y);
        arm.setGoalState(goalState);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
