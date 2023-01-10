package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DriveBase.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DriveBase.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    //FL, FR, BL, BR
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DriveBase.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DriveBase.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DriveBase.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DriveBase.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DriveBase.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DriveBase.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DriveBase.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DriveBase.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private static final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    //private final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator m_estimator;

    SwerveModuleState[] m_states;
    ChassisSpeeds m_speeds;

    public SwerveDrive() {
        Mk4ModuleConfiguration configuration = new Mk4ModuleConfiguration();
        configuration.setDriveCurrentLimit(Constants.DriveBase.driveCurrentLimit);
        configuration.setSteerCurrentLimit(Constants.DriveBase.rotCurrentLimit);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(0, 0),
                configuration,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.Modules.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.Modules.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.Modules.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.Modules.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(1, 0),
                configuration,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.Modules.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.Modules.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.Modules.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.Modules.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(2, 0),
                configuration,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.Modules.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.Modules.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.Modules.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.Modules.BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(1, 3).withPosition(3, 0),
                configuration,
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.Modules.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.Modules.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.Modules.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.Modules.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_estimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), getKinematics(),
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.01), // Gyroscope rotation std-dev
                VecBuilder.fill(0.15, 0.15, 0.01)); // Vision (x, y, rotation) std-devs

        m_states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        //tab.add("Field", m_field).withSize(4,2).withPosition(4, 0);

        tab.addNumber("Odometry X", () -> getPosition().getX()).withPosition(0, 4);
        tab.addNumber("Odometry Y", () -> getPosition().getY()).withPosition(1, 4);
        tab.addNumber("Odometry Angle", () -> getPosition().getRotation().getDegrees()).withPosition(2, 4);
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees()).withPosition(3, 4);
    }

    public void setPosition(Pose2d m_position) {
        m_estimator.resetPosition(m_position, getGyroscopeRotation());
    }

    public void addVisionMeasurement(Pose2d m_observed, double time){
        m_estimator.addVisionMeasurement(m_observed, time);
    }

    public void addPath(String name, PathPlannerTrajectory m_trajectory) {
        //m_field.getObject(name).setTrajectory(m_trajectory);
    }

    public Pose2d getPosition() {
        return m_estimator.getEstimatedPosition();
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getVelocity() {return m_speeds;}

    public double getMaxVelocityMetersPerSecond(){
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    public double getMaxAcceleration(){
        return getMaxVelocityMetersPerSecond() * 2;
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public static Rotation2d getGyroscopeRotation() {
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_speeds = chassisSpeeds;
        m_states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    }

    public void setModuleStates(SwerveModuleState[] states){
        m_states = states;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);



        m_estimator.update(getGyroscopeRotation(), m_states[0], m_states[1],
                m_states[2], m_states[3]);

        Pose2d robotPose = getPosition();


        //m_field.setRobotPose(getPosition());

        m_frontLeftModule.set(m_states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveBase.MAX_VOLTAGE, m_states[0].angle.getRadians());
        m_frontRightModule.set(m_states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveBase.MAX_VOLTAGE, m_states[1].angle.getRadians());
        m_backLeftModule.set(m_states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveBase.MAX_VOLTAGE, m_states[2].angle.getRadians());
        m_backRightModule.set(m_states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveBase.MAX_VOLTAGE, m_states[3].angle.getRadians());

    }



}
