package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;

import java.util.function.DoubleSupplier;

public class ArmOverride extends CommandBase {

    private final Arm arm;
    private final DoubleSupplier getx1, gety1, mult1;

    public ArmOverride(Arm arm, DoubleSupplier getx1, DoubleSupplier gety1, DoubleSupplier mult1) {
        this.arm = arm;
        this.getx1 = getx1;
        this.gety1 = gety1;
        this.mult1 = mult1;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setIsOpenLoop(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setGoalState(arm.getCurrentState());
        arm.setInput(getx1.getAsDouble(), gety1.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.setIsOpenLoop(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
