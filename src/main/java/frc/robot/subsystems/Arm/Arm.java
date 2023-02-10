// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmState;

import java.util.function.BiFunction;

import static frc.robot.constants.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;
    private final RelativeEncoder m_motor1Encoder;
    private final RelativeEncoder m_motor2Encoder;
    private ArmState goalState;
    private double input1;
    private double input2;
    private double last_velocity1;
    private double last_velocity2;
    private DoublePublisher currentTheta1;
    private DoublePublisher currentTheta2;
    private  DoublePublisher omega1;
    private  DoublePublisher omega2;
    private  DoublePublisher alpha1;
    private  DoublePublisher alpha2;
    private  DoublePublisher targetTheta1;
    private  DoublePublisher targetTheta2;
    private DoublePublisher error2;
    private  DoublePublisher appliedOutput1;
    private  DoublePublisher appliedOutput2;
    private DoublePublisher feedForwardOutput1;
    private DoublePublisher feedForwardOutput2;
    private DoublePublisher kfilter1;
    private DoublePublisher kfilter2;
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final ProfiledPIDController controller2;
    private final ProfiledPIDController controller1;
    private final ArmFeedforward armFeedForward2;
    private final ArmFeedforward armFeedForward1;
    private boolean isOpenLoop;


    private final BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (armState, inputs) -> dynamics(new ArmState(armState), inputs.get(0,0), inputs.get(1,0)).getStateMatrixDot();
    private final BiFunction<edu.wpi.first.math.Matrix<N4, edu.wpi.first.math.numbers.N1>, edu.wpi.first.math.Matrix<N2, N1>, Matrix<N4, N1>> h = (mat1, mat2) -> new MatBuilder<>(Nat.N4(), Nat.N1()).fill(1, 0, 0, 0);
    private final ExtendedKalmanFilter<N4, N2, N4> kalmanFilter = new ExtendedKalmanFilter<>(
            Nat.N4(),
            Nat.N2(),
            Nat.N4(),
            f,
            h,
            VecBuilder.fill(0.1, 0.1, 0.2, 0.2), //State stds
            VecBuilder.fill(0.01, 0.01, 0.05, 0.05), // Measurement stds
            Matrix::minus,
            Matrix::plus,
            0.02 //dt
    );

    /** Creates a new Arm. */
    public Arm() {
        controller2 = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(4, 8));
        controller1 = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2.5, 3));

        armFeedForward2 = new ArmFeedforward(0.05, 0.25, 0.19, 0.01);
        armFeedForward1 = new ArmFeedforward(0.0, .36, 3.9, .03);

        m_motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor1.setInverted(false);
        m_motor2.setInverted(false);
        m_motor1Encoder = m_motor1.getEncoder();
        m_motor2Encoder= m_motor2.getEncoder();

        m_motor1Encoder.setPositionConversionFactor(S1);
        m_motor2Encoder.setPositionConversionFactor(S2);

        m_motor1Encoder.setVelocityConversionFactor(S1 / 60);
        m_motor2Encoder.setVelocityConversionFactor(S2 / 60);

        m_motor1Encoder.setPosition(Units.degreesToRadians(90));
        m_motor2Encoder.setPosition(Units.degreesToRadians(270));
        goalState = new ArmState(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(270), 0, 0, 0, 0);
        last_velocity1 = 0;
        last_velocity2 = 0;

        configureDashBoard();
        isOpenLoop = false;
    }

    /**
     * Configures NetworkTables Publishers and Subscribers
     */
    public void configureDashBoard() {
        currentTheta1 =  networkTable.getDoubleTopic("Current theta1").publish();
        currentTheta2 =  networkTable.getDoubleTopic("Current theta2").publish();

        omega1 =  networkTable.getDoubleTopic("Current omega1").publish();
        omega2 =  networkTable.getDoubleTopic("Current omega2").publish();

        alpha1  =  networkTable.getDoubleTopic("Current alpha").publish();
        alpha2 =  networkTable.getDoubleTopic("Current alpha2").publish();

        targetTheta1 =  networkTable.getDoubleTopic("targetTheta1").publish();
        targetTheta2 =  networkTable.getDoubleTopic("Target Theta2").publish();

        error2 = networkTable.getDoubleTopic("Error 2").publish();

        appliedOutput1 =  networkTable.getDoubleTopic("Current appliedOutput1").publish();
        appliedOutput2 =  networkTable.getDoubleTopic("Current appliedOutput2").publish();

        feedForwardOutput1 = networkTable.getDoubleTopic("Feed Forward1").publish();
        feedForwardOutput2 = networkTable.getDoubleTopic("FeedForward2").publish();

        kfilter1 = networkTable.getDoubleTopic("kfilter1").publish();
        kfilter2 = networkTable.getDoubleTopic("kfilter2").publish();
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

        kfilter1.set(kalmanFilter.getXhat(0));
        kfilter2.set(kalmanFilter.getXhat(1));
    }

    /**
     * Gets the current goal position of the arm
     * @return Goal ArmState
     */
    public ArmState getGoalState() {
        return goalState;
    }

    /**
     * Sets arm Goal State only called by Commands
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
     * @return Arm State
     */
    public ArmState getCurrentState() {
        Rotation2d theta1 = Rotation2d.fromRadians(m_motor1Encoder.getPosition());
        Rotation2d theta2 = Rotation2d.fromRadians(m_motor2Encoder.getPosition());

        double omega1 = m_motor1Encoder.getVelocity();
        double omega2 = m_motor2Encoder.getVelocity();
        double firstAcceleration = (m_motor1Encoder.getVelocity() - last_velocity1) / (0.02);
        double secondAcceleration = (m_motor2Encoder.getVelocity() - last_velocity2) / (0.02);
        last_velocity1 = m_motor1Encoder.getVelocity();
        last_velocity2 = m_motor2Encoder.getVelocity();

        return new ArmState(theta1, theta2, omega1, omega2, firstAcceleration, secondAcceleration);
    }

    /**
     * Sets Motor Voltages for the Arm
     * @param volt1 voltage for proximal motor
     * @param volt2 voltage for distal motor
     */
    public void setVoltages(double volt1, double volt2) {
        m_motor1.setVoltage(volt1);
        m_motor2.setVoltage(volt2);
    }

    /**
     * Sets inputs and then adds feedForwards to apply to the motors
     * @param i1 motor1
     * @param i2 motor2
     */
    public void setInput(double i1, double i2) {
        input1 = i1;
        input2 = i2;
    }
    private Matrix<N2, N2> getDynamicMatrixM(ArmState state) {


        double theta2 = state.getTheta2();
        double M1 = m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2) + I1 + I2 + 2 * m2 * r1 * l2 * Math.cos(theta2);
        double M2 = m2 * r2 * r2 + I2 + m2 * l1 * r2 * Math.cos(theta2);
        double M4 = m2 * r2 * r2 + I2;

        return new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                M1,
                M2,
                M2,
                M4
        );


    }

    /**
     * Represents centrifugal and coriolis terms
     * @param state ArmState
     * @return Coriolis and Centrifugal Matrix
     */
    public Matrix<N2, N2> getDynamicMatrixC(ArmState state) {

        double theta2 = state.getTheta2();
        double omega1 = state.getOmega1(), omega2 = state.getOmega2();


        double C1 = -m2 * l1 * r2 * Math.sin(theta2) * omega2;
        double C2 = -m2 * l1 * r2 * Math.sin(theta2) * (omega2 + omega1);
        double C3 = -m2 * l1 * r2 * Math.sin(theta2);
        double C4 = 0;

        return new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                C1,
                C2,
                (C3),
                C4
        );

    }

    /**
     * Torque applied by gravity
     * @param state ArmState
     * @return Torque Matrix
     */
    public Matrix<N2, N1> getDynamicMatrixG(ArmState state) {
        double theta1 = state.getTheta1(), theta2 = state.getTheta2();
        double G1 = (m1 * r1 + m2 * l1) * g * Math.cos(theta1) * m2 * r2 * g * Math.cos(theta1 + theta2);
        double G2  = m2 * r2 * g * Math.cos(theta1 + theta2);

        return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                G1,
                G2
        );

    }

    /**
     * returns the time derivatives of the state
     * @param state ArmState
     * @param inputVolt1 input to motor 1
     * @param inputVolt2 input to motor 2
     * @return state-dot
     */
    public ArmState dynamics(ArmState state, double inputVolt1, double inputVolt2) {
        Matrix<N2, N1> omegas = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                state.getOmega1(),
                state.getOmega2()
        );
        Matrix<N2, N1> inputs = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                inputVolt1,
                inputVolt2
        );
        Matrix<N2, N1> basic_torque = K3.times(inputs);
        Matrix<N2, N2> M = getDynamicMatrixM(state);
        Matrix<N2, N1> G = getDynamicMatrixG(state);
        Matrix<N2, N2> C = getDynamicMatrixC(state);

        Matrix<N2, N1>  back_emf_loss = K4.times(omegas);


        Matrix<N2, N1> torque = basic_torque.minus(back_emf_loss);
        Matrix<N2, N1> accels = M.inv().times(torque.minus(C.times(omegas).minus(G)));

        return new ArmState(state.getTheta1(), state.getTheta2(), state.getOmega1(), state.getOmega2(), accels.get(0,0), accels.get(1,0));
    }

    /**
     * Integrates inverse kinematics, state space estimation, feedForward, PID feedback, and pathfinding to navigate to a goal set by a command
     * theta time derivatives are calculated via a jacobian to the inverse kinematics function
     */
    @Override
    public void periodic() {
        if(!isOpenLoop) {
            ArmState goalState = getGoalState();
            input1 = controller1.calculate(getCurrentState().getTheta1(), goalState.getTheta1());
            input2 = controller2.calculate(getCurrentState().getTheta2(), goalState.getTheta2());
        }

            double betterFeedForward1 = armFeedForward1.calculate(controller1.getSetpoint().position, controller1.getSetpoint().velocity);
            double betterFeedForward2 = armFeedForward2.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity);

            setVoltages(MathUtil.clamp(input1 + betterFeedForward1, -12, 12), MathUtil.clamp(input2 + betterFeedForward2, -12, 12));

        updateDashBoard();
        kalmanFilter.correct(VecBuilder.fill(input1 + betterFeedForward1, input2 + betterFeedForward2), getCurrentState().getStateMatrix4());
    }
}
