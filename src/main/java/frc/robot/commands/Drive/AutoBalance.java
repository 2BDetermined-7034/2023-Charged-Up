package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemLogging;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase implements SubsystemLogging {
    private final SwerveDrive m_swerveDrive;

    PIDController pid = new PIDController(1.2, 0, 0.1);

    public AutoBalance(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        pid.setTolerance(Math.toRadians(5));
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

        double speed = pid.calculate(m_swerveDrive.getPitch().getRadians(), 0);
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Pitch", m_swerveDrive.getPitch().getDegrees());
        if (Math.abs(speed) < 3 && Math.abs(m_swerveDrive.getPitch().getDegrees()) < 5) {
            speed = 0;
        }
        m_swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                speed,
                0,
                0,
                SwerveDrive.getGyroscopeRotation()
        ));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
