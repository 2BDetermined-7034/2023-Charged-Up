// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.util.ArmState;

import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {

  private final Arm arm;


  /** Creates a new ArmCommand. */
  public DefaultArmCommand(Arm arm, DoubleSupplier getx1, DoubleSupplier gety1 ) {
    this.arm = arm;
    double x = getx1.getAsDouble() * Constants.ArmConstants.fullRadius;
    double y = gety1.getAsDouble() * Constants.ArmConstants.fullRadius;

    if(x == 0 && y == 0) return;

   ArmState invkinematics = Arm.inverseKinematics(x, y, true);

    arm.setGoalState(invkinematics);

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
    return false;
  }
}
