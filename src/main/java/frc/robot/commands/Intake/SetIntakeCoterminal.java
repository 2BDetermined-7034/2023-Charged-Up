
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class SetIntakeCoterminal extends CommandBase {
    private final Intake m_intake;

    public SetIntakeCoterminal(Intake intake) {
        m_intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
