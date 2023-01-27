// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;

public class ArmCommand extends CommandBase {

  private final Arm arm;

  private static final Arm.ArmState extendedState = new Arm.ArmState(Arm.inverseKinematics(new double[] {1.5, 3}));

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
    Matrix<N2, N1> ffs = arm.feedForward(extendedState);

    arm.setVoltages(
            controller1.calculate(arm.getThetaValues()[0], extendedState.theta1) + ffs.get(1, 1),
            controller2.calculate(arm.getThetaValues()[1], extendedState.theta2) + ffs.get(2, 1)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setVoltages(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Arm.ArmState currentState = arm.getState();
    return Math.abs(extendedState.theta1 - currentState.theta1) <= 4 && Math.abs(extendedState.theta2 - currentState.theta2) <= 4;
  }
}
