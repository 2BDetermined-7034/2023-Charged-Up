package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SetPosition extends CommandBase {
    private final SwerveDrive m_swerveDrive;
    Pose2d pose2d;

    public SetPosition(SwerveDrive swerveDrive, Pose2d pose2d) {
        m_swerveDrive = swerveDrive;
        this.pose2d = pose2d;
        addRequirements(m_swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_swerveDrive.setPosition(pose2d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
