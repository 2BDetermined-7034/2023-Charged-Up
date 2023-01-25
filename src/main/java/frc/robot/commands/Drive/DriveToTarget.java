package frc.robot.commands.Drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

import java.util.function.Consumer;
import java.util.function.Supplier;


public class DriveToTarget extends CommandBase {

    SwerveDrive m_swerve;
    VisionLocking m_locker;
    PathPlannerTrajectory m_trajectory;


    private final Timer timer = new Timer();
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final PPHolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final PIDController secondaryController;


    public DriveToTarget(SwerveDrive m_swerve, VisionLocking m_locker) {
        this.m_swerve = m_swerve;
        this.m_locker = m_locker;
        this.poseSupplier = m_swerve::getPosition;
        this.kinematics = m_swerve.getKinematics();
        this.controller = new PPHolonomicDriveController(
                new PIDController(0.1, 0.001, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0.1, 0.001, 0), // Y controller (usually the same values as X controller)
                new PIDController(0.0, 0, 0)
        );
        this.outputModuleStates = m_swerve::setModuleStates;
        this.secondaryController = new PIDController(0, 0, 0);
        addRequirements(m_swerve, m_locker);
    }

    @Override
    public void initialize() {
        m_trajectory = PathFactory.createTagPath(m_swerve.getPosition(), m_locker.getLockedPosition());
        m_swerve.addTrajectory(m_trajectory);
        //m_swerve.setLimeLightDriver();
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {

        if(!m_swerve.getVision().isTargetAvailable()) {
            if(m_swerve.getVision().getPipeLine() != 1) {
                m_swerve.getVision().setPipeLine(1);
            }

            //Do Something with retro-reflective tape Here

            float KpAim = -0.1f;
            float KpDistance = -0.1f;
            float min_aim_command = 0.05f;


            double tx = m_swerve.getVision().getHorizontalOffset();
            double ty = m_swerve.getVision().getVerticalOffset();
            double steeringAdjust = 0;

            double heading_error = -tx;
            double distance_error = -ty;
            double steering_adjust = 0.0;

            if (tx > 1.0)
            {
                steering_adjust = KpAim*heading_error - min_aim_command;
            }
            else if (tx < -1.0)
            {
                steering_adjust = KpAim*heading_error + min_aim_command;
            }

            double distance_adjust = KpDistance * distance_error;


            ChassisSpeeds speeds = new ChassisSpeeds(distance_adjust, 0, steeringAdjust);

            SwerveModuleState[] targetModuleStates =
                    this.kinematics.toSwerveModuleStates(speeds);

            this.outputModuleStates.accept(targetModuleStates);

            return;
        }

        double currentTime = this.timer.get();
        PathPlannerTrajectory.PathPlannerState desiredState = (PathPlannerTrajectory.PathPlannerState) this.m_trajectory.sample(currentTime);

        Pose2d currentPose = this.poseSupplier.get();
        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

        SwerveModuleState[] targetModuleStates =
                this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        this.outputModuleStates.accept(targetModuleStates);



    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.m_trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
        m_swerve.setLimeLightVision();
        m_swerve.getVision().setPipeLine(0);
        if (interrupted) {
            this.outputModuleStates.accept(
                    this.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));

        }
    }
}
