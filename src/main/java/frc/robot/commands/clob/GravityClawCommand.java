package frc.robot.commands.clob;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GravityClawSubsystem;


public class GravityClawCommand extends CommandBase {
    private final GravityClawSubsystem gravityClawSubsystem;
    private final boolean direction;

    public GravityClawCommand(GravityClawSubsystem gravityClawSubsystem, boolean direction) {
        this.gravityClawSubsystem = gravityClawSubsystem;
        this.direction = direction;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.gravityClawSubsystem);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        gravityClawSubsystem.setSolonoid(direction);

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
