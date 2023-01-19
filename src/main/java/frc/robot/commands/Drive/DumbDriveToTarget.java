package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.VisionLocking;


public class DumbDriveToTarget extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final VisionLocking visionLocking;
    private final PIDController xController;
    private final PIDController yController;

    public DumbDriveToTarget(SwerveDrive swerveDrive, VisionLocking visionLocking) {
        this.swerveDrive = swerveDrive;
        this.visionLocking = visionLocking;
        this.xController = new PIDController(0.5,0.01,0);
        this.yController = new PIDController(0.5,0.01,0);

        addRequirements(this.swerveDrive, this.visionLocking);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = maxSpeed(xController.calculate(Math.round(swerveDrive.getPosition().getX()), visionLocking.getLockedPosition().getX()));
        double ySpeed = maxSpeed(yController.calculate(Math.round(swerveDrive.getPosition().getY()), visionLocking.getLockedPosition().getY()));
        swerveDrive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, 0));
    }

    public double maxSpeed(double speed){
        speed = MathUtil.clamp(speed, -swerveDrive.getMaxSpeed(), swerveDrive.getMaxSpeed());
        return speed;
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
