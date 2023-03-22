package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveModule;
import swervelib.math.SwerveModuleState2;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class YAGSLswerve extends SubsystemBase implements SubsystemLogging {

    private SwerveDrive m_swerveDrive;
    private LimeLight m_limelight;
    private double speedMulti;
    private SwerveModuleState2[] m_desiredStates;


    public YAGSLswerve() throws IOException {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.



        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        this.m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
        this.m_limelight = new LimeLight();
        this.speedMulti = 1;
        m_desiredStates = m_swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        m_swerveDrive.field = new Field2d();
    }

    @Override
    public void updateLogging() {
        log("Pose2D", getPosition());
        log("Swerve Module States", m_swerveDrive.getStates());
        log("Speed Multi", speedMulti);
    }

    public Rotation2d getGyroscopeRotation() {
        return m_swerveDrive.getYaw();
    }

    public double getMaxSpeed() {
        return m_swerveDrive.getModules()[0].getConfiguration().maxSpeed;
    }

    public Pose2d getPosition() {
        return m_swerveDrive.getPose();
    }

    public void setSpeedMulti(double speed) {
        speed = MathUtil.clamp(speed, 0, 1);
        speedMulti = speed;
    }

    public void updateOdometry() {
        m_swerveDrive.updateOdometry();
        if(m_limelight.isTargetAvailable()){
            log("Vision Pose", m_limelight.getBotPose().toPose2d());
            m_swerveDrive.addVisionMeasurement(m_limelight.getBotPose().toPose2d(), Timer.getFPGATimestamp(), true, 0.2);
        }
    }

    public SwerveDriveKinematics getKinematics() {
        return m_swerveDrive.kinematics;
    }

    public LimeLight getLimeLight() {
        return m_limelight;
    }

    public ChassisSpeeds getVelocity() {
        return m_swerveDrive.getRobotVelocity();
    }

    public void zeroGyroscope() {
        m_swerveDrive.zeroGyro();
    }
    
    public SwerveModulePosition[] getModulePosition() {
        return m_swerveDrive.getModulePositions();
    }
    
    public Rotation2d getPitch() {
        return m_swerveDrive.getPitch();
    }

    public Rotation2d getYaw() {
        return m_swerveDrive.getYaw();
    }

    public Rotation2d getRoll() {
        return m_swerveDrive.getRoll();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.setChassisSpeeds(chassisSpeeds);
        m_swerveDrive.setModuleStates(m_swerveDrive.kinematics.toSwerveModuleStates(chassisSpeeds), true);
    }
    
    public void stop() {
        var m_speeds = new ChassisSpeeds(0,0,0);
        m_swerveDrive.setModuleStates(m_swerveDrive.kinematics.toSwerveModuleStates(m_speeds), true);
    }

    public void setModuleStates(SwerveModuleState2[] states, boolean isOpenLoop) {
        m_swerveDrive.setModuleStates(states, isOpenLoop);
    }

    public void setModuleStates1(SwerveModuleState[] states, boolean isOpenLoop){
        m_swerveDrive.setModuleStates((SwerveModuleState2[]) states, isOpenLoop);
    }


    public Optional<Transform2d> getCamTransform() {
        if (m_limelight.isTargetAvailable()) return Optional.ofNullable(m_limelight.getCamTransform2d());
        return Optional.empty();
    }

    public void addTrajectory(PathPlannerTrajectory m_trajectory) {
        m_swerveDrive.field.getObject("traj").setTrajectory(m_trajectory);
    }



    @Override
    public void periodic() {
        updateOdometry();
        m_swerveDrive.field.setRobotPose(getPosition());

        boolean m_isOpenLoop = false;

        for(int i = 0; i < 4; i++){
            m_desiredStates[i].speedMetersPerSecond *= speedMulti;
        }

        setModuleStates(m_desiredStates, true);

        SmartDashboard.putNumber("Pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Roll", getRoll().getDegrees());
    }

    public void setPosition(Pose2d pose2d) {
        m_swerveDrive.resetOdometry(pose2d);
    }

    public void setLimeLightVision() {
        m_limelight.setModeVision();
    }
}

