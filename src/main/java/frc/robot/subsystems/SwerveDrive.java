package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.SwerveModule;
import frc.robot.constants.COTSSwerveConstants;
import frc.robot.constants.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {

    //FL, FR, BL, BR
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.Drivebase.Measurements.width / 2.0, Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(Constants.Drivebase.Measurements.width / 2.0, -Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(-Constants.Drivebase.Measurements.width / 2.0, Constants.Drivebase.Measurements.length / 2.0),
            new Translation2d(-Constants.Drivebase.Measurements.width / 2.0, -Constants.Drivebase.Measurements.length / 2.0)
    );

    private static final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SwerveDrivePoseEstimator m_estimator;
    private boolean m_IsOpenLoop = false;

    private final Vision m_vision = new Vision();

    SwerveModuleState[] m_states;
    ChassisSpeeds m_speeds;

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
                VecBuilder.fill(0.02, 0.02, 0.01), // estimator values (x, y, rotation) std-devs
                VecBuilder.fill(0.15, 0.15, 0.01)
        );

        m_states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        //tab.add("Field", m_field).withSize(4,2).withPosition(4, 0);

        tab.addNumber("Odometry X", () -> getPosition().getX()).withPosition(0, 4);
        tab.addNumber("Odometry Y", () -> getPosition().getY()).withPosition(1, 4);
        tab.addNumber("Odometry Angle", () -> getPosition().getRotation().getDegrees()).withPosition(2, 4);
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees()).withPosition(3, 4);
    }

    public void setPosition(Pose2d m_position) {
        zeroGyroscope();
        m_estimator.resetPosition(
                getGyroscopeRotation(),
                getModulePosition(),
                m_position
        );
    }

    public void addVisionMeasurement(Pose2d m_observed, double time){
        m_estimator.addVisionMeasurement(m_observed, time);
    }

    public double getMaxSpeed() {
        return m_frontLeftModule.cotsSwerveConstants.maxSpeed;
    }
    public Pose2d getPosition() {
        return m_estimator.getEstimatedPosition();
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getVelocity() {return m_speeds;}

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        };
    }

    public static Rotation2d getGyroscopeRotation() {
        /*
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());

         */
        return Rotation2d.fromDegrees(360 - m_navx.getYaw());
    }


    public void drive(ChassisSpeeds chassisSpeeds) {
        m_speeds = chassisSpeeds;
        m_states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    }

    public void setModuleStates(SwerveModuleState[] states){
        m_states = states;
    }

    public void updateOdometry() {
        m_estimator.update(getGyroscopeRotation(), new SwerveModulePosition[] {
                    m_frontLeftModule.getPosition(),
                    m_frontRightModule.getPosition(),
                    m_backLeftModule.getPosition(),
                    m_backRightModule.getPosition()
                });
        Pair<Pose2d, Double> result = m_vision.getEstimatedGlobalPose(m_estimator.getEstimatedPosition());

            Pose2d camPose = result.getFirst();
            m_estimator.addVisionMeasurement(camPose, result.getSecond());
    }
    

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, getMaxSpeed());

        m_estimator.update(
                getGyroscopeRotation(),
                getModulePosition()
        );

        m_frontLeftModule.setDesiredState(m_states[0], m_IsOpenLoop);
        m_frontRightModule.setDesiredState(m_states[1], m_IsOpenLoop);
        m_backLeftModule.setDesiredState(m_states[2], m_IsOpenLoop);
        m_backRightModule.setDesiredState(m_states[3], m_IsOpenLoop);
    }

}