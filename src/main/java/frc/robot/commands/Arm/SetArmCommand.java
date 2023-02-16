// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.util.ArmState;

public class SetArmCommand extends CommandBase {
    private final double theta1;
    private final double theta2;
    private final Arm arm;

    /**
     * Creates a new ArmCommand.
     */
    public SetArmCommand(Arm arm, double theta1, double theta2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.arm = arm;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmState goalState = new ArmState(theta1, theta2);

        arm.setGoalState(goalState);
    }

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
        return arm.atSetPoint();
    }
}
