package frc.robot.commands.Drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.YAGSLswerve;

import java.util.function.DoubleSupplier;

public class HeadingDriveCommand extends CommandBase {
    private final YAGSLswerve m_swerveDrive;
    private final double heading;
    private final PIDController m_pidController;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public HeadingDriveCommand(YAGSLswerve drivetrainSubsystem,
                               double heading,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_swerveDrive = drivetrainSubsystem;
        this.heading = heading;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        m_pidController = new PIDController(0.04, 0, 0);
        m_pidController.enableContinuousInput(-180, 180);
        m_pidController.setTolerance(3);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double headingSpeed = m_pidController.calculate(SwerveDrive.getGyroscopeRotation().getDegrees(), heading);
        headingSpeed = Math.abs(headingSpeed) < 0.1 || m_pidController.atSetpoint() ? 0 : headingSpeed;
        m_swerveDrive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        headingSpeed,
                        SwerveDrive.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_rotationSupplier.getAsDouble()) > 0.05;
    }
}