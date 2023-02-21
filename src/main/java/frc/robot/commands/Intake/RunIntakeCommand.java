package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;


public class RunIntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier reverseSpeed;
    private final Indexer index;

    public RunIntakeCommand(Intake intake, Indexer index,  DoubleSupplier forwardSpeed, DoubleSupplier reverseSpeed) {
        m_intake = intake;
        this.index = index;
        this.forwardSpeed = forwardSpeed;
        this.reverseSpeed = reverseSpeed;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intake);
        addRequirements(index);
    }

    @Override
    public void initialize() {
        m_intake.setSolenoid(true);

    }

    @Override
    public void execute() {
//        if(setCoterminal) {
//            m_intake.setCoterminal();
//        } else {
//            m_intake.runIntakeForward();
//        }
        index.runIndexerClockwise();
        m_intake.runIntake(forwardSpeed.getAsDouble(), reverseSpeed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.runIntake(0,0);
        index.stopIndexer();

        m_intake.setCoterminal();
        m_intake.setSolenoid(false);
    }
}
