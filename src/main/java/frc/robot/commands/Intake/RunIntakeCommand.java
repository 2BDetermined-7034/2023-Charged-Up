package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;


public class RunIntakeCommand extends CommandBase {
    private final SwerveDrive m_drive;
    private final Intake m_intake;
    private final DoubleSupplier intakeSpeed;
    private final DoubleSupplier indexerSpeed;

    private final Indexer index;
    private final boolean fireSolenoid;

    public RunIntakeCommand(SwerveDrive drive, Intake intake, Indexer index, DoubleSupplier intakeSpeed, DoubleSupplier indexerSpeed, boolean fireSoelenoid) {
        this.m_drive = drive;
        m_intake = intake;
        this.index = index;
        this.intakeSpeed = intakeSpeed;
        this.indexerSpeed = indexerSpeed;
        this.fireSolenoid = fireSoelenoid;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(intake);
        addRequirements(index);
    }

    @Override
    public void initialize() {
        if (intakeSpeed.getAsDouble() <= 0.3) m_drive.setSpeedMulti(0.2);
        else if (intakeSpeed.getAsDouble() >= 0.3) m_drive.setSpeedMulti(0.4);

        if(fireSolenoid) {
            m_intake.setSolenoid(false);
        }
    }

    @Override
    public void execute() {
        m_intake.runIntake(intakeSpeed.getAsDouble());
        index.runIndexer(indexerSpeed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setSpeedMulti(1);
        m_intake.runIntake(0);
        index.stopIndexer();

        m_intake.setSolenoid(true);
    }
}
