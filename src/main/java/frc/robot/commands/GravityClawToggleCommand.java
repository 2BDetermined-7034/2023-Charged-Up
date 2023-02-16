package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GravityClawSubsystem;


public class GravityClawToggleCommand extends CommandBase {
    private final GravityClawSubsystem gravityClawSubsystem;

    public GravityClawToggleCommand(GravityClawSubsystem gravityClawSubsystem) {
        this.gravityClawSubsystem = gravityClawSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.gravityClawSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
gravityClawSubsystem.ToggleSolonoid();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
