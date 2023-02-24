package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;

public class RunIndexerCommand extends CommandBase {
    private final Indexer m_indexer;

    public RunIndexerCommand(Indexer indexer) {
        m_indexer = indexer;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_indexer.runIndexer(Constants.Intake.indexerSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stopIndexer();
    }
}
