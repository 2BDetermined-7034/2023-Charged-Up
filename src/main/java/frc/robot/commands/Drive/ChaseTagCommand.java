package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 1);
  

  private final SwerveDrive swerveDrive;
  private final VisionLocking visionLocking;

  private final ProfiledPIDController xController = new ProfiledPIDController(0.2, 0.1, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.2, 0.1, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.3, 0.01, 0, OMEGA_CONSTRAINTS);

  public ChaseTagCommand( 
        SwerveDrive swerveDrive,
        VisionLocking visionLocking) {
    this.swerveDrive = swerveDrive;
    this.visionLocking = visionLocking;

    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    xController.setIntegratorRange(-1, 1);
    yController.setIntegratorRange(-1, 1);

    omegaController.setTolerance(Units.degreesToRadians(1.5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setIntegratorRange(-1, 1);

      SmartDashboard.putData(xController);
      SmartDashboard.putData(yController);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    var robotPose = swerveDrive.getLimeLight().getBotPose().toPose2d();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
        var robotPose = swerveDrive.getPosition();
        var goalPose = visionLocking.getLockedPosition();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());

      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal() || !swerveDrive.getLimeLight().isTargetAvailable()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal() || !swerveDrive.getLimeLight().isTargetAvailable()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, SwerveDrive.getGyroscopeRotation()));
  }
  public boolean isFinished() {
    return omegaController.atGoal() && xController.atGoal() && yController.atGoal();
  }
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

}