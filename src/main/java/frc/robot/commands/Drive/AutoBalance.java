package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoBalance extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    PIDController pid = new PIDController(1.5,0,0.1);

    public AutoBalance(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        pid.setTolerance(6);
        pid.setIntegratorRange(-3, 3);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_swerveDrive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                -pid.calculate(m_swerveDrive.getRoll().getRadians(), 0),
                0,
                0,
                SwerveDrive.getGyroscopeRotation()
        ));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                0,
                SwerveDrive.getGyroscopeRotation()
        ));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}