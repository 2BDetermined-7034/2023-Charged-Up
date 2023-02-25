// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.util.ArmState;

import java.util.function.BooleanSupplier;

public class SetArmCommand extends CommandBase {
    private final ArmState goalState;
    private final Arm arm;
    private final Intake intake;
    private final boolean toggleIntakeOnEnd;

    /**
     * Creates a new ArmCommand.
     */
    public SetArmCommand(Arm arm, Intake intake, ArmState goalState, boolean toggleIntakeOnEnd) {
        this.goalState = goalState;
        this.arm = arm;
        this.intake = intake;
        this.toggleIntakeOnEnd = toggleIntakeOnEnd;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        arm.setGoalState(goalState);
        intake.setSolenoid(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(toggleIntakeOnEnd) intake.toggleSolenoid();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.isArmAtSetpoint();
    } // when arm is at setpoint
}
