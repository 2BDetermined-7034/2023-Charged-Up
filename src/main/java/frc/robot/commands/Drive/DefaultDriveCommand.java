package frc.robot.commands.Drive;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final Arm m_Arm;

    public DefaultDriveCommand(SwerveDrive drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               Arm m_Arm) {
        this.m_swerveDrive = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_Arm = m_Arm;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_swerveDrive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0.25 * m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        SwerveDrive.getGyroscopeRotation()
                )
        );


        if(DriverStation.isTeleop() && m_Arm.isArmHome()) {
            m_swerveDrive.setSpeedMulti(0.2);
        }
        else {
            m_swerveDrive.setSpeedMulti(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_swerveDrive.setSpeedMulti(1);
    }
}