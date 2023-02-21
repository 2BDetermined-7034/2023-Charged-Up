// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmState;

public class SetArmCommand extends CommandBase {
    private final double theta1;
    private final double theta2;
    private final Arm arm;
    private final Intake intake;

    /**
     * Creates a new ArmCommand.
     */
    public SetArmCommand(Arm arm, Intake intake, double theta1, double theta2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.arm = arm;
        this.intake = intake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmState goalState = new ArmState(theta1, theta2);

        arm.setGoalState(goalState);
        intake.setSolenoid(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setSolenoid(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.isArmAtSetpoint();
    } // when arm is at setpoint
}
