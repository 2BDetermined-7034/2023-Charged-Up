package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import static frc.robot.constants.Constants.ArmConstants.*;

public class ArmOverride extends CommandBase {

    protected final Arm arm;
    protected final DoubleSupplier getx1, gety1, enable;

    public ArmOverride(Arm arm, DoubleSupplier getx1, DoubleSupplier gety1, DoubleSupplier enable) {
        this.arm = arm;
        this.getx1 = getx1;
        this.gety1 = gety1;
        this.enable = enable;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setGoalState(arm.getCurrentState().clear());
        arm.setIsOpenLoop(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setInput(getx1.getAsDouble() * kMaxArmOverrideSpeedShoulder, gety1.getAsDouble() * kMaxArmOverrideSpeedDistal);
        arm.setGoalState(arm.getCurrentState().clear());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setGoalState(arm.getCurrentState().clear());
        arm.setIsOpenLoop(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return 0.05 > enable.getAsDouble();
    }
}
