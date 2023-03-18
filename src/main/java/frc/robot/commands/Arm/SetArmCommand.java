// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.ArmState;

public class SetArmCommand extends CommandBase {
    private final ArmState goalState;
    private final Arm arm;
    private final SwerveDrive drive;
    private final boolean toggleSpeedOnEnd;

    /**
     * Creates a new ArmCommand.
     */
    public SetArmCommand(Arm arm, SwerveDrive drive, ArmState goalState, boolean toggleSpeedOnEnd) {
        this.goalState = goalState;
        this.arm = arm;
        this.toggleSpeedOnEnd = toggleSpeedOnEnd;
        this.drive = drive;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.setSpeedMulti(0.2);
        arm.setGoalState(goalState);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(toggleSpeedOnEnd) {
            drive.setSpeedMulti(1);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.isArmAtSetpoint();
    } // when arm is at setpoint
}
