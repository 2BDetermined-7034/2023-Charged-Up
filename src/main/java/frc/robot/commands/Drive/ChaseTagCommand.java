package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;

import java.util.function.Supplier;

public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  

  private final SwerveDrive swerveDrive;
  private final VisionLocking visionLocking;

  private final Supplier<Pose2d> pose2dSupplier;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private int lastTargetID;

  public ChaseTagCommand( 
        SwerveDrive swerveDrive,
        VisionLocking visionLocking) {
    this.swerveDrive = swerveDrive;
    this.visionLocking = visionLocking;
    pose2dSupplier = swerveDrive::getPosition;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    lastTargetID = -1;
    var robotPose = pose2dSupplier.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = pose2dSupplier.get();

    if (swerveDrive.getLimeLight().isTargetAvailable()) {
      // Find the tag we want to chase

      // This is new target data, so recalculate the goal
        lastTargetID = (int) swerveDrive.getLimeLight().getTargetID();
        
        // Transform the robot's pose to find the camera's pose

        // Transform the tag's pose to set our goal
        var goalPose = visionLocking.getLockedPosition();


        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      
    }

    //TODO fix this part for Swerve
    
    if(!(lastTargetID == -1)) {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      swerveDrive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  

}