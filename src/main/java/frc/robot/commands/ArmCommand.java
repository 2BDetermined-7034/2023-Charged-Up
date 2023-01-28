// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.util.ArmState;

public class ArmCommand extends CommandBase {

    private final Arm arm;

    private static final PIDController controller1 = new PIDController(0, 0, 0);
    private static final PIDController controller2 = new PIDController(0, 0, 0);

    /** Creates a new ArmCommand. */
    public ArmCommand(Arm arm) {
        this.arm = arm;
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
        arm.setVoltages(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        ArmState currentState = arm.getCurrentState();
        return Math.abs(arm.getGoalState().theta1.getRadians() - currentState.theta1.getRadians()) <= Units.degreesToRadians(4)
                && Math.abs(arm.getGoalState().theta1.getRadians() - currentState.theta2.getRadians()) <= Units.degreesToRadians(4);
    }
}