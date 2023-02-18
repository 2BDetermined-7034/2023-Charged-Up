package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    PIDController pid = new PIDController(1.4, 0, 0.2);

    public AutoBalance(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        pid.setTolerance(6, 5);
        pid.setIntegratorRange(-3, 3);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_swerveDrive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                -pid.calculate(m_swerveDrive.getRoll().getRadians(), 0),
                0,
                0,
                SwerveDrive.getGyroscopeRotation()
        ));

        SmartDashboard.putBoolean("isAtSetpoint", pid.atSetpoint());
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
