package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoBalance extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    PIDController pid = new PIDController(1.3,0,0.1);

    public AutoBalance(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        pid.setTolerance(0.1);
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
//        return Math.abs(m_swerveDrive.getRoll().getDegrees()) <= 2;
        return false;
    }
}
