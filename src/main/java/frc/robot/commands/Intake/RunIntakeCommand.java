package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RunIntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final boolean setCoterminal;

    public RunIntakeCommand(Intake intake, boolean setCoterminal) {
        m_intake = intake;
        this.setCoterminal = setCoterminal;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(setCoterminal) {
            m_intake.setCoterminal();
        } else {
            m_intake.runIntakeForward();
        }
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
