package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.COTSSwerveConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.SwerveModuleConstants;
import frc.robot.util.SwerveModule;

public class SwerveDrive extends SubsystemBase {

    public static final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    //FL, FR, BL, BR
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.Drivebase.Measurements.width / 2.0, Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(Constants.Drivebase.Measurements.width / 2.0, -Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(-Constants.Drivebase.Measurements.width / 2.0, Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(-Constants.Drivebase.Measurements.width / 2.0, -Constants.Drivebase.Measurements.length / 2.0)
    );
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SwerveDrivePoseEstimator m_estimator;
    private final Field2d m_field;
    SwerveModuleState[] m_states;
    ChassisSpeeds m_speeds;
    private LimeLight limeLight = new LimeLight();

    public SwerveDrive() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        zeroGyroscope();
        m_frontLeftModule = new SwerveModule(
                0,
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(0, 0),
                new SwerveModuleConstants(
                        Constants.Drivebase.MotorIDs.flDrive,
                        Constants.Drivebase.MotorIDs.flSteer,
                        Constants.Drivebase.MotorIDs.flEncoder,
                        Constants.Drivebase.MotorIDs.flOffset
                ),
                COTSSwerveConstants.SDSMK4i(Constants.Drivebase.Measurements.driveRatio)
        );

        m_frontRightModule = new SwerveModule(
                1,
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(1, 0),
                new SwerveModuleConstants(
                        Constants.Drivebase.MotorIDs.frDrive,
                        Constants.Drivebase.MotorIDs.frSteer,
                        Constants.Drivebase.MotorIDs.frEncoder,
                        Constants.Drivebase.MotorIDs.frOffset
                ),
                COTSSwerveConstants.SDSMK4i(Constants.Drivebase.Measurements.driveRatio)
        );

        m_backLeftModule = new SwerveModule(
                2,
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(2, 0),
                new SwerveModuleConstants(
                        Constants.Drivebase.MotorIDs.blDrive,
                        Constants.Drivebase.MotorIDs.blSteer,
                        Constants.Drivebase.MotorIDs.blEncoder,
                        Constants.Drivebase.MotorIDs.blOffset
                ),
                COTSSwerveConstants.SDSMK4i(Constants.Drivebase.Measurements.driveRatio)
        );

        m_backRightModule = new SwerveModule(
                3,
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(3, 0),
                new SwerveModuleConstants(
                        Constants.Drivebase.MotorIDs.brDrive,
                        Constants.Drivebase.MotorIDs.brSteer,
                        Constants.Drivebase.MotorIDs.brEncoder,
                        Constants.Drivebase.MotorIDs.brOffset
                ),
                COTSSwerveConstants.SDSMK4i(Constants.Drivebase.Measurements.driveRatio)
        );

        m_estimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroscopeRotation(),
                getModulePosition(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.9, 0.9, 0.9)
        );


        m_field = new Field2d();
        m_states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        limeLight.setModeVision();


        tab.add(m_field).withPosition(4, 0).withSize(5, 4);
        tab.addNumber("Odometry X", () -> getPosition().getX()).withPosition(0, 4);
        tab.addNumber("Odometry Y", () -> getPosition().getY()).withPosition(1, 4);
        tab.addNumber("Odometry Angle", () -> getPosition().getRotation().getDegrees()).withPosition(2, 4);
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees()).withPosition(3, 4);
        tab.addNumber("Gyro Roll", () -> getRoll().getRadians()).withPosition(3, 4);
        tab.addBoolean("Tag", () -> limeLight.isTargetAvailable()).withPosition(4, 5);
    }

    public static Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - m_navx.getYaw());
    }

    public void addVisionMeasurement(Pose2d m_observed, double time) {
        m_estimator.addVisionMeasurement(m_observed, time);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(m_navx.getRoll());
    }

    public double getMaxSpeed() {
        return m_frontLeftModule.cotsSwerveConstants.maxSpeed;
    }

    public Pose2d getPosition() {
        //Logger.getInstance().recordOutput("Position", m_estimator.getEstimatedPosition());
        return m_estimator.getEstimatedPosition();
    }

    public void setPosition(Pose2d m_position) {
        zeroGyroscope();
        m_estimator.resetPosition(
                getGyroscopeRotation(),
                getModulePosition(),
                m_position
        );
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public LimeLight getLimeLight() {
        return limeLight;
    }

    public ChassisSpeeds getVelocity() {
        return m_speeds;
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[]{
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        };
    }

    public void setLimeLightDriver() {
        limeLight.setModeDriver();
    }

    public void setLimeLightVision() {
        limeLight.setModeVision();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_speeds = chassisSpeeds;
        m_states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    }

    public void stop() {
        m_speeds = new ChassisSpeeds(0, 0, 0);
        m_states = m_kinematics.toSwerveModuleStates(m_speeds);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        m_states = states;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, getMaxSpeed());

        m_estimator.update(
                getGyroscopeRotation(),
                getModulePosition()
        );

        updateOdometry();
        m_field.setRobotPose(getPosition());

        boolean m_IsOpenLoop = false;
        m_frontLeftModule.setDesiredState(m_states[0], m_IsOpenLoop);
        m_frontRightModule.setDesiredState(m_states[1], m_IsOpenLoop);
        m_backLeftModule.setDesiredState(m_states[2], m_IsOpenLoop);
        m_backRightModule.setDesiredState(m_states[3], m_IsOpenLoop);
    }

    private void updateOdometry() {
        if (limeLight.isTargetAvailable()) {
            addVisionMeasurement(limeLight.getBotPose().toPose2d(), Timer.getFPGATimestamp());
        } else return;
    }

    public void addTrajectory(PathPlannerTrajectory m_trajectory) {
        m_field.getObject("traj").setTrajectory(m_trajectory);
    }

    public Transform2d getCamTransform() {
        if (limeLight.isTargetAvailable()) return limeLight.getCamTransform2d();
        return new Transform2d();
    }

    public void resetOdometry(Pose2d pose) {
        m_estimator.resetPosition(getGyroscopeRotation(), getModulePosition(), pose);
    }

}