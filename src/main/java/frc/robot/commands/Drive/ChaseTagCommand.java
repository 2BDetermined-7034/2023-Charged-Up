package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class ChaseTagCommand extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);


    private final SwerveDrive swerveDrive;
    private final VisionLocking visionLocking;

    private final Supplier<Pose2d> pose2dSupplier;
    double xSpeed;
    double ySpeed;
    double omegaSpeed;

    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

    public ChaseTagCommand(
            SwerveDrive swerveDrive,
            VisionLocking visionLocking
    ) {
        this.swerveDrive = swerveDrive;
        this.visionLocking = visionLocking;
        pose2dSupplier = swerveDrive::getPosition;

        xController.setTolerance(0.4);
        yController.setTolerance(0.25);
        omegaController.setTolerance(Units.degreesToRadians(10));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {

        swerveDrive.getLimeLight().setModeVision();
        var robotPose = pose2dSupplier.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {

        if(!DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("X error", xController.getPositionError());
            SmartDashboard.putNumber("Y error", yController.getPositionError());
            SmartDashboard.putNumber("T error", omegaController.getPositionError());
            SmartDashboard.putBoolean("Is Finished", yController.atGoal() && xController.atGoal() && omegaController.atGoal());
        }


        var robotPose = pose2dSupplier.get();
        var goalPose = visionLocking.getLockedPosition();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());

        // Drive to the target
        xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        swerveDrive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation())
        );

    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return xSpeed == 0 && ySpeed == 0 && omegaSpeed == 0;
    }

}