// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemLogging;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.*;

public class Arm extends SubsystemBase implements SubsystemLogging {
    private final CANSparkMax m_motor1, m_motor2;
    private final RelativeEncoder m_motor1Encoder, m_motor2Encoder;
    private final Encoder m_AbsoluteEncoder1, m_AbsoluteEncoder2;
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final ProfiledPIDController controller1, controller2;
    private final ArmFeedforward armFeedForward1, armFeedForward2;
    private ArmState goalState;
    private double input1, input2;
    private double last_velocity1, last_velocity2;
    private DoublePublisher currentTheta1, currentTheta2, omega1, omega2, alpha1, alpha2, targetTheta1, targetTheta2, error2, appliedOutput1, appliedOutput2, feedForwardOutput1, feedForwardOutput2;
    private boolean isOpenLoop;

    /**
     * Creates a new Arm.
     */
    public Arm() {
        controller2 = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4, 8));
        controller1 = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2.5, 3));

        controller1.setTolerance(Math.toRadians(2));
        controller2.setTolerance(Math.toRadians(2));

        armFeedForward2 = new ArmFeedforward(0.01, kG1, kV1, kA1);
        armFeedForward1 = new ArmFeedforward(0.0, kG2, kV2, kA2);

        m_motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor1.setInverted(false);
        m_motor2.setInverted(false);
        m_motor1Encoder = m_motor1.getEncoder();
        m_motor2Encoder = m_motor2.getEncoder();

        m_motor1Encoder.setPositionConversionFactor(S1);
        m_motor2Encoder.setPositionConversionFactor(S2);

        m_motor1Encoder.setVelocityConversionFactor(S1 / 60);
        m_motor2Encoder.setVelocityConversionFactor(S2 / 60);

        m_motor1Encoder.setPosition(Units.degreesToRadians(90));
        m_motor2Encoder.setPosition(Units.degreesToRadians(320));

        //DIO encoders
        m_AbsoluteEncoder1 = new Encoder(EncoderChannelA1,EncoderChannelB1, false, CounterBase.EncodingType.k2X);
        m_AbsoluteEncoder2 = new Encoder(EncoderChannelA2,EncoderChannelB2, false, CounterBase.EncodingType.k2X);
        m_AbsoluteEncoder1.setSamplesToAverage(6);
        m_AbsoluteEncoder2.setSamplesToAverage(6);
        m_AbsoluteEncoder1.setDistancePerPulse(S1);
        m_AbsoluteEncoder2.setDistancePerPulse(S2);

        goalState = new ArmState(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(270), 0, 0, 0, 0);
        last_velocity1 = 0;
        last_velocity2 = 0;

        isOpenLoop = false;

        configureDashBoard();

    }

    /**
     * Configures NetworkTables Publishers and Subscribers
     */
    public void configureDashBoard() {
        currentTheta1 = networkTable.getDoubleTopic("Current theta1").publish();
        currentTheta2 = networkTable.getDoubleTopic("Current theta2").publish();

        omega1 = networkTable.getDoubleTopic("Current omega1").publish();
        omega2 = networkTable.getDoubleTopic("Current omega2").publish();

        alpha1 = networkTable.getDoubleTopic("Current alpha").publish();
        alpha2 = networkTable.getDoubleTopic("Current alpha2").publish();

        targetTheta1 = networkTable.getDoubleTopic("targetTheta1").publish();
        targetTheta2 = networkTable.getDoubleTopic("Target Theta2").publish();

        error2 = networkTable.getDoubleTopic("Error 2").publish();

        appliedOutput1 = networkTable.getDoubleTopic("Current appliedOutput1").publish();
        appliedOutput2 = networkTable.getDoubleTopic("Current appliedOutput2").publish();

        feedForwardOutput1 = networkTable.getDoubleTopic("Feed Forward1").publish();
        feedForwardOutput2 = networkTable.getDoubleTopic("FeedForward2").publish();
    }


    /**
     * Updates NetworkTables Publishers
     */
    public void updateDashBoard() {
        currentTheta1.set(Units.radiansToDegrees(getCurrentState().getTheta1()));
        currentTheta2.set(Units.radiansToDegrees(getCurrentState().getTheta2()));


        omega1.set(Units.radiansToDegrees(getCurrentState().getOmega1()));
        omega2.set(Units.radiansToDegrees(getCurrentState().getOmega2()));

        alpha1.set(Units.radiansToDegrees(getCurrentState().getAlpha1()));
        alpha2.set(Units.radiansToDegrees(getCurrentState().getAlpha2()));

        targetTheta1.set(Units.radiansToDegrees(getGoalState().getTheta1()));
        targetTheta2.set(Units.radiansToDegrees(getGoalState().getTheta2()));

        error2.set(Units.radiansToDegrees(controller2.getPositionError()));

        appliedOutput1.set(m_motor1.getAppliedOutput());
        appliedOutput2.set(m_motor2.getAppliedOutput());

        feedForwardOutput1.set(armFeedForward1.calculate(controller1.getSetpoint().position, controller1.getSetpoint().velocity, 0));
        feedForwardOutput2.set(armFeedForward2.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity, 0));
    }

    /**
     * AdvantageKit Logging
     */
    @Override
    public void updateLogging() {
        log("theta1", getCurrentState().getTheta1());
        log("theta2", getCurrentState().getTheta2());
        log("omega1", getCurrentState().getOmega1());
        log("omega2", getCurrentState().getOmega2());
        log("Applied Output1", m_motor1.getAppliedOutput());
        log("Applied Output2", m_motor2.getAppliedOutput());
        log("error1", controller1.getPositionError());
        log("error2", controller2.getPositionError());
    }

    /**
     * Gets the current goal position of the arm
     *
     * @return Goal ArmState
     */
    public ArmState getGoalState() {
        return goalState;
    }

    /**
     * Sets arm Goal State only called by Commands
     *
     * @param goalState ArmState to which the arm will go to
     */
    public void setGoalState(ArmState goalState) {
        this.goalState = goalState;
    }

    public void setIsOpenLoop(boolean condition) {
        this.isOpenLoop = condition;
        controller1.reset(new TrapezoidProfile.State(getCurrentState().getTheta1(), getCurrentState().getOmega1()));
        controller2.reset(new TrapezoidProfile.State(getCurrentState().getTheta2(), getCurrentState().getOmega2()));
    }

    /**
     * TODO set state-space estimation
     * <p>
     * Gets the current state of the arm
     *
     * @return Arm State
     */
    public ArmState getCurrentState() {
        Rotation2d theta1 = Rotation2d.fromRadians(m_motor1Encoder.getPosition());
        Rotation2d theta2 = Rotation2d.fromRadians(m_motor2Encoder.getPosition());

        double omega1 = m_motor1Encoder.getVelocity();
        double omega2 = m_motor2Encoder.getVelocity();
        double firstAlpha = (m_motor1Encoder.getVelocity() - last_velocity1) / (0.02);
        double secondAlpha = (m_motor2Encoder.getVelocity() - last_velocity2) / (0.02);
        last_velocity1 = m_motor1Encoder.getVelocity();
        last_velocity2 = m_motor2Encoder.getVelocity();
//
//        Rotation2d theta1 = Rotation2d.fromRadians(m_AbsoluteEncoder1.getDistance());
//        Rotation2d theta2 = Rotation2d.fromRadians(m_AbsoluteEncoder2.getDistance());
//        double omega1 = m_AbsoluteEncoder1.getRate(), omega2 = m_AbsoluteEncoder2.getRate();
//        double firstAlpha = (omega1 - last_velocity1) / 0.02, secondAlpha = (omega2 - last_velocity2) / 0.02;
//        last_velocity1 = omega1;
//        last_velocity2 = omega2;

        return new ArmState(theta1, theta2, omega1, omega2, firstAlpha, secondAlpha);
    }

    /**
     * Checks wheather the arm is at the home position
     * @return boolean isArmAtHomeOrAwayOnALongJourneyToFindItself'sLifePurposeAndFulfullItsDestinyOfWorkingAtObama'sFriedChicken
     */
    public boolean isArmHome() {
        return Math.abs(getCurrentState().getTheta1() - Math.toRadians(90)) < Math.toRadians(4) && Math.abs(getCurrentState().getTheta2() - Math.toRadians(320)) < Math.toRadians(4);
    }
    public boolean isArmAtSetpoint() {
        return controller1.atGoal() && controller2.atGoal();
    }

    /**
     * Sets Motor Voltages for the Arm
     *
     * @param volt1 voltage for proximal motor
     * @param volt2 voltage for distal motor
     */
    public void setVoltages(double volt1, double volt2) {
        m_motor1.setVoltage(volt1);
        m_motor2.setVoltage(volt2);
    }

    /**
     * Sets inputs and then adds feedForwards to apply to the motors
     *
     * @param i1 motor1
     * @param i2 motor2
     */
    public void setInput(double i1, double i2) {
        input1 = i1;
        input2 = i2;
    }

    /**
     * Integrates inverse kinematics, state space estimation, feedForward, PID feedback, and pathfinding to navigate to a goal set by a command
     * theta time derivatives are calculated via a jacobian to the inverse kinematics function
     */
    @Override
    public void periodic() {
        if (!isOpenLoop) {
            ArmState goalState = getGoalState();
            input1 = controller1.calculate(getCurrentState().getTheta1(), goalState.getTheta1());
            input2 = controller2.calculate(getCurrentState().getTheta2(), goalState.getTheta2());
        }

        double betterFeedForward1 = armFeedForward1.calculate(controller1.getSetpoint().position, controller1.getSetpoint().velocity);
        double betterFeedForward2 = armFeedForward2.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity);

        setVoltages(MathUtil.clamp(input1 + betterFeedForward1, -12, 12), MathUtil.clamp(input2 + betterFeedForward2, -12, 12));

        updateDashBoard();
        updateLogging();
    }


}