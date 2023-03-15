package frc.robot.commands.ActiveGrabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ActiveGrabberSubsystem;


public class ActiveGrabberMotor extends CommandBase {
    private final ActiveGrabberSubsystem activeGrabberSubsystem;
    private final double volts;

    public ActiveGrabberMotor(ActiveGrabberSubsystem activeGrabberSubsystem, double volts) {
        this.activeGrabberSubsystem = activeGrabberSubsystem;
        this.volts = volts;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.activeGrabberSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        activeGrabberSubsystem.runGrabberMotor(volts);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        activeGrabberSubsystem.stopGrabberMotor();

    }
}
