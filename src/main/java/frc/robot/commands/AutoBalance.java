package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {

    private final SwerveDrive m_swerveDrive;
    ShuffleboardTab tab = Shuffleboard.getTab("AutoBalance");
    private double finalSpeed;
    private double pitch;
    private double roll;

    /**
     * Creates a new AutoBalance.
     */
    public AutoBalance(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
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
        // I love direction matrices :D
        // First convert pitch, yaw, and roll into dimension matricies
        // Then multiply the matrices in order yaw, pitch, roll
        // Then apply the z column to vector k and you have the normal vector of the robot
        // Deviation from vector k is equivalent to the angle of elevation

        //double yaw = Math.toRadians(m_navx.getYaw());
        pitch = Math.toRadians(m_swerveDrive.getPitch());
        roll = Math.toRadians(m_swerveDrive.getRoll());

        int mult = 1;

        double[] normalVector = {
                Math.sin(pitch),
                -Math.sin(roll) * Math.cos(pitch),
                Math.cos(pitch) * Math.cos(roll)
        };
        if (normalVector[0] > 0) {
            mult = -1;
        }

        finalSpeed = mult * 0.01 * Math.toDegrees(
                Math.atan2(
                        Math.sqrt(Math.pow(normalVector[1], 2) + Math.pow(normalVector[0], 2)), normalVector[2]
                )
        ); // Meth

        m_swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                finalSpeed,
                0,
                0,
                SwerveDrive.getGyroscopeRotation()
        ));

        tab.addNumber("Balance Speed", () -> finalSpeed);
        tab.addNumber("Pitch", () -> m_swerveDrive.getPitch());
        tab.addNumber("Roll", () -> m_swerveDrive.getRoll());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
