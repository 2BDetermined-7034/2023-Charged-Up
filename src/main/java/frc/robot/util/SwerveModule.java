package frc.robot.util;

import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.COTSSwerveConstants;
import frc.robot.constants.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class SwerveModule {
    public final int moduleNumber;
    private final ShuffleboardLayout dashboard;
    public final COTSSwerveConstants cotsSwerveConstants;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;
    private final CANSparkMax mAngleMotor;
    private final CANSparkMax mDriveMotor;
    private final CANCoder absoluteEncoder;

    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mAngleEncoder;
    private final SparkMaxPIDController drivePIDController;
    private final SparkMaxPIDController mAnglePIDController;

    private SwerveModuleState targetState;

    SimpleMotorFeedforward feedforward;

    private double simSpeedCache;
    private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

    public SwerveModule(int moduleNumber, ShuffleboardLayout container, SwerveModuleConstants moduleConstants, COTSSwerveConstants moduleRatios){
        this.moduleNumber = moduleNumber;
        this.dashboard = container;

        configureDashboard();

        this.cotsSwerveConstants = moduleRatios;
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Encoder */
        absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleEncoder = mAngleMotor.getEncoder();
        mAnglePIDController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive motor */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        drivePIDController = mDriveMotor.getPIDController();
        configDriveMotor();

        targetState = new SwerveModuleState(0,Rotation2d.fromDegrees(0));

        feedforward = new SimpleMotorFeedforward(cotsSwerveConstants.driveKS, cotsSwerveConstants.driveKV, cotsSwerveConstants.driveKA);

        lastAngle = getState().angle;
    }

    private void configureDashboard() {
        dashboard.addDouble("Speed", () -> mDriveMotor.getAppliedOutput());
        dashboard.addDouble("Absolute Angle", () -> getAbsoluteAngle().getDegrees());
        dashboard.addDouble("Relative Angle", () -> getAngle().getDegrees());
        dashboard.addDouble("Target Angle", () -> getTargetState().angle.getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
        targetState = desiredState;

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        simSpeedCache = desiredState.speedMetersPerSecond;
        simAngleCache = desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / cotsSwerveConstants.maxSpeed;
            //mDriveMotor.set(percentOutput);
            mDriveMotor.setVoltage(percentOutput*12);
        }
        else {
            mDriveMotor.getPIDController().setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (cotsSwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents bad.
        mAngleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(mAngleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).minus(angleOffset);
    }

    private void resetToAbsolute(){
        mAngleEncoder.setPosition(getAbsoluteAngle().getDegrees());
    }

    private void configAngleEncoder(){
        absoluteEncoder.configFactoryDefault();
        CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = cotsSwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        absoluteEncoder.configAllSettings(swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Constants.Drivebase.MotorConfig.angleSmartCurrentLimit);
        mAngleMotor.setSecondaryCurrentLimit(Constants.Drivebase.MotorConfig.angleSecondaryCurrentLimit);
        mAngleMotor.setInverted(cotsSwerveConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Drivebase.MotorConfig.angleNeutralMode);

        mAngleEncoder.setPositionConversionFactor((1/cotsSwerveConstants.angleGearRatio) // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
                * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        mAnglePIDController.setP(cotsSwerveConstants.angleKP);
        mAnglePIDController.setI(cotsSwerveConstants.angleKI);
        mAnglePIDController.setD(cotsSwerveConstants.angleKD);
        mAnglePIDController.setFF(cotsSwerveConstants.angleKF);
    }

    private void configDriveMotor(){
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setSmartCurrentLimit(Constants.Drivebase.MotorConfig.driveSmartCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(Constants.Drivebase.MotorConfig.driveSecondaryCurrentLimit);
        mDriveMotor.setInverted(cotsSwerveConstants.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Drivebase.MotorConfig.driveNeutralMode);
        mDriveMotor.setOpenLoopRampRate(Constants.Drivebase.MotorConfig.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(Constants.Drivebase.MotorConfig.closedLoopRamp);

        mDriveEncoder.setPositionConversionFactor(1 / cotsSwerveConstants.driveGearRatio
                * cotsSwerveConstants.wheelCircumference
        );
        mDriveEncoder.setPosition(0);

        drivePIDController.setP(cotsSwerveConstants.driveKP);
        drivePIDController.setI(cotsSwerveConstants.driveKI);
        drivePIDController.setD(cotsSwerveConstants.driveKD);
        drivePIDController.setFF(cotsSwerveConstants.driveKF);

    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
        );
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(mDriveEncoder.getPosition(), getAbsoluteAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }
}