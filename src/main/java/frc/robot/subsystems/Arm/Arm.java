// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.*;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemLogging;

import java.util.function.BiFunction;

import static frc.robot.constants.Constants.ArmConstants.*;

public class Arm extends SubsystemBase implements SubsystemLogging {
    private final CANSparkMax m_motor1, m_motor2;
    private final RelativeEncoder m_motor1Encoder, m_motor2Encoder;
    private final DutyCycleEncoder m_AbsoluteEncoder1, m_AbsoluteEncoder2;
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final ProfiledPIDController controller1, controller2;
    private final ArmFeedforward armFeedForward1, armFeedForward2;
    private ArmState goalState;
    private double input1, input2;
    private double last_velocity1, last_velocity2;
    private DoublePublisher currentTheta1, currentTheta2, omega1, omega2, alpha1, alpha2, targetTheta1, targetTheta2, error2, appliedOutput1, appliedOutput2, feedForwardOutput1, feedForwardOutput2;
    private boolean isOpenLoop;
    public final BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, edu.wpi.first.math.numbers.N1>> simulateFunc = (Matrix<N4, N1> x, Matrix<N2, N1> u) -> dynamics.simulate((Vector<N4>) x, (Vector<edu.wpi.first.math.numbers.N2>) u, 0.02).extractColumnVector(0);
    private final ExtendedKalmanFilter<N4, N2, N4> kFilter =
            new ExtendedKalmanFilter<>(
            Nat.N4(),
            Nat.N2(),
            Nat.N4(),
            dynamics.accelFunction(),
            dynamics.simulateStepFunc(),
            VecBuilder.fill(0.1, 0.1, 0.2, 0.2),
            VecBuilder.fill(0.02, 0.02, 0.03, 0.03),
            Matrix::plus,
            Matrix::minus,
            0.02);


    /**
     * Creates a new Arm.
     */
    public Arm() {
        controller2 = new ProfiledPIDController(10, 1, 0.1, new TrapezoidProfile.Constraints(1.5, 3));
        controller1 = new ProfiledPIDController(3.5, 0.5, 0.05, new TrapezoidProfile.Constraints(4, 5));

        controller2.setIntegratorRange(-2, 2);
        controller1.setIntegratorRange(-2, 2);

        controller1.setTolerance(Math.toRadians(6), 1.5);
        controller2.setTolerance(Math.toRadians(6), 1.5);

        armFeedForward2 = new ArmFeedforward(0.01, kG1, kV1, kA1);
        armFeedForward1 = new ArmFeedforward(0.01, kG2, kV2, kA2);

        m_motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor1.setSmartCurrentLimit(10);
        m_motor2.setSmartCurrentLimit(10);

        m_motor1.setSecondaryCurrentLimit(40);
        m_motor2.setSecondaryCurrentLimit(40);

        m_motor1.setInverted(true);
        m_motor2.setInverted(false);

        setModeBreak();

        m_motor1Encoder = m_motor1.getEncoder();
        m_motor2Encoder = m_motor2.getEncoder();

        m_motor1Encoder.setPositionConversionFactor(S1);
        m_motor2Encoder.setPositionConversionFactor(S2);

        m_motor1Encoder.setVelocityConversionFactor(S1 / 60);
        m_motor2Encoder.setVelocityConversionFactor(S2 / 60);

        //DIO encoders
        m_AbsoluteEncoder1 = new DutyCycleEncoder(8);
        m_AbsoluteEncoder2 = new DutyCycleEncoder(9);
        m_AbsoluteEncoder1.setDistancePerRotation(-360);
        m_AbsoluteEncoder2.setDistancePerRotation(-360);

        m_AbsoluteEncoder1.setPositionOffset(kEncoder1Offset);
        m_AbsoluteEncoder2.setPositionOffset(kEncoder2Offset);

        last_velocity1 = 0;
        last_velocity2 = 0;

        setIsOpenLoop(false);
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
        log("theta1", Units.radiansToDegrees(getCurrentState().getTheta1()));
        log("theta2", Units.radiansToDegrees(getCurrentState().getTheta2()));
        log("motor theta1", m_motor1Encoder.getPosition());
        log("motor theta2", m_motor2Encoder.getPosition());
        log("encoder theta1", m_AbsoluteEncoder1.getDistance());
        log("encoder theta2", m_AbsoluteEncoder2.getDistance() + m_AbsoluteEncoder1.getDistance());
        log("Target Theta2", Units.radiansToDegrees(goalState.getTheta2()));
        log("Target Theta1", Units.radiansToDegrees(goalState.getTheta1()));
        log("omega1", getCurrentState().getOmega1());
        log("omega2", getCurrentState().getOmega2());
        log("Applied Output1", m_motor1.getAppliedOutput());
        log("Applied Output2", m_motor2.getAppliedOutput());
        log("error1", controller1.getPositionError());
        log("error2", controller2.getPositionError());
        log("newFF1", dynamics.feedforward(getCurrentState().getPositionVector(), getCurrentState().getOmegaVector()).get(0,1));
        log("newFF2", dynamics.feedforward(getCurrentState().getPositionVector(), getCurrentState().getOmegaVector()).get(0,1));
        log("kFilter Theta1", kFilter.getXhat(0));
        log("kFilter Theta2", kFilter.getXhat(1));


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

        Rotation2d theta1 = Rotation2d.fromDegrees(m_AbsoluteEncoder1.getDistance());
        Rotation2d theta2 = Rotation2d.fromDegrees(m_AbsoluteEncoder2.getDistance() + m_AbsoluteEncoder1.getDistance());

        double omega1 = m_motor1Encoder.getVelocity();
        double omega2 = m_motor2Encoder.getVelocity();

        double firstAlpha = (m_motor1Encoder.getVelocity() - last_velocity1) / (0.02);
        double secondAlpha = (m_motor2Encoder.getVelocity() - last_velocity2) / (0.02);

        last_velocity1 = m_motor1Encoder.getVelocity();
        last_velocity2 = m_motor2Encoder.getVelocity();

        return new ArmState(theta1, theta2, omega1, omega2, firstAlpha, secondAlpha);
    }

    public boolean goalStateValid() {
        return Math.toDegrees(goalState.getTheta1()) < 140 && Math.toDegrees(goalState.getTheta2()) < 360;
    }

    /**
     * Checks wheather the arm is at the home position
     * @return boolean isArmAtHomeOrAwayOnALongJourneyToFindItself'sLifePurposeAndFulfullItsDestinyOfWorkingAtObama'sFriedChicken
     */
    public boolean isArmHome() {
        return Math.abs(getCurrentState().getTheta1() - Math.toRadians(90)) < Math.toRadians(4) && Math.abs(getCurrentState().getTheta2() - Math.toRadians(275)) < Math.toRadians(4);
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
     * Changes Idle Mode of Motors
     */
    public void setModeBreak() {
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Changes Idle Mode of Motors
     */
    public void setModeCoast() {
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Integrates inverse kinematics, state space estimation, feedForward, PID feedback, and A* pathfinding to navigate to a goal set by a command
     * theta time derivatives are calculated via a jacobian to the inverse kinematics function
     */
    @Override
    public void periodic() {

        if (!goalStateValid()) {
            goalState = getCurrentState();
            setVoltages(0, 0);
        }
        ArmState goalState = getGoalState();
        if (!isOpenLoop) {
            TrapezoidProfile.State profile1 = new TrapezoidProfile.State(goalState.getTheta1(), goalState.getOmega1());
            TrapezoidProfile.State profile2 = new TrapezoidProfile.State(goalState.getTheta2(), goalState.getOmega2());
            input1 = controller1.calculate(getCurrentState().getTheta1(), profile1);
            input2 = controller2.calculate(getCurrentState().getTheta2(), profile2);
        }

        double feedForwardShoulder = armFeedForward1.calculate(controller1.getSetpoint().position, controller1.getSetpoint().velocity);
        double feedForwardElbow = armFeedForward2.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity);

        //Vector<edu.wpi.first.math.numbers.N2> ff = dynamics.feedforward(getCurrentState().getPositionVector(), getCurrentState().getOmegaVector());

        setVoltages(MathUtil.clamp(input1 + feedForwardShoulder, -12, 12), MathUtil.clamp(input2 + feedForwardElbow, -12, 12));


        kFilter.correct(VecBuilder.fill(input1 + feedForwardShoulder, input2 + feedForwardElbow), getCurrentState().getStateMatrix4());
        updateDashBoard();
        updateLogging();
    }

}
