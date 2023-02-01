// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.*;

//Check out this https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
//Copied from here


public class Arm extends SubsystemBase {

    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;
    private final RelativeEncoder m_motor1Encoder;
    private final RelativeEncoder m_motor2Encoder;
    private ArmState goalState;
    private double lastinput1;
    private double lastinput2;


    private final PIDController controller1;
    private final PIDController controller2;

    /** Creates a new Arm. */
    public Arm() {
        controller1 = new PIDController(0.001, 0, 0);
        controller2 = new PIDController(0.1, 0.01, 0);

        controller1.enableContinuousInput(0, 2 * Math.PI);
        controller2.enableContinuousInput(0, 2 * Math.PI);

        controller1.setIntegratorRange(-1, 1);
        controller2.setIntegratorRange(-3, 3);

        m_motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor1.setInverted(true);
        m_motor2.setInverted(true);
        m_motor1Encoder = m_motor1.getEncoder();
        m_motor2Encoder= m_motor2.getEncoder();

        m_motor1Encoder.setPositionConversionFactor(S1);
        m_motor2Encoder.setPositionConversionFactor(S2);

        m_motor1Encoder.setVelocityConversionFactor(S1 / 60);
        m_motor2Encoder.setVelocityConversionFactor(S2 / 60);

        m_motor1Encoder.setPosition(Units.degreesToRadians(90));
        m_motor2Encoder.setPosition(Units.degreesToRadians(180) + m_motor1Encoder.getPosition());
        goalState = new ArmState(Rotation2d.fromDegrees(m_motor1Encoder.getPosition()), Rotation2d.fromRadians(m_motor2Encoder.getPosition()), 0, 0);

        configureDashBoard();
    }

    public void configureDashBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addDouble("Current theta1", () -> getCurrentState().theta1.getDegrees()).withPosition(0,0);
        tab.addDouble("Current theta2", () -> getCurrentState().theta2.getDegrees()).withPosition(1, 0);
        tab.addDouble("theta 1 error", ()-> Units.radiansToDegrees(controller1.getPositionError()));
        tab.addDouble("theta 2 error", ()-> Units.radiansToDegrees(controller2.getPositionError()));
        tab.addDouble("omega1", () -> getCurrentState().omega1).withPosition(2, 0);
        tab.addDouble("omega2", () -> getCurrentState().omega2).withPosition(3, 0);
        tab.addDouble("Alpha1", () -> getCurrentState().accel1).withPosition(4, 0);
        tab.addDouble("Alpha2", () -> getCurrentState().accel2).withPosition(5, 0);

        tab.addDouble("Target theta1", () -> getGoalState().theta1.getDegrees()).withPosition(0,1);
        tab.addDouble("Target theta2", () -> getGoalState().theta2.getDegrees()).withPosition(1,1);

        tab.addDouble("motor1 appliedOutput", () -> m_motor1.getAppliedOutput());
        tab.addDouble("motor2 appliedOutput", () -> m_motor2.getAppliedOutput());

        tab.add("M1 controller", controller1);
        tab.add("M2 controller", controller2);
    }
    public ArmState getGoalState() {
        return goalState;
    }
    public void setGoalState(ArmState state) {
        goalState = state;
    }
    public ArmState getCurrentState() {
        Rotation2d theta1 = Rotation2d.fromRadians(normalizeAngle(m_motor1Encoder.getPosition()));
        Rotation2d theta2 = Rotation2d.fromRadians(normalizeAngle(m_motor2Encoder.getPosition() - theta1.getRadians()));

        double omega1 = m_motor1Encoder.getVelocity();
        double omega2 = m_motor2Encoder.getVelocity();

        return getAccels(new ArmState(theta1, theta2, omega1, omega2), lastinput1, lastinput2);
    }

    /**
     *
     * @param x end effector pos x (m)
     * @param y end effector pos y (m)
     * @return ArmState identical to position
     */
    public static ArmState inverseKinematics(double x, double y, boolean invert) {
        Rotation2d theta2 = Rotation2d.fromRadians(Math.acos((x * x + y * y - (l1 * l1 + l2 * l2)) / (2 * l1 * l2)));
        Rotation2d theta1 = Rotation2d.fromRadians(Math.atan2(y, x) + (invert ? -1 : 1) * Math.atan2(l2 * Math.sin(theta2.getRadians()), l1 + l2 * Math.cos(theta2.getRadians())));

        return new ArmState(theta1, theta2, 0,0);
    }

    /**
     * gets the inertia matrix
     * @param state ArmState
     * @return inertia matrix at the current state
     */
    private Matrix<N2, N2> getDynamicMatrixM(ArmState state) {
        double theta2 = state.theta2.getRadians();
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
        double theta2 = state.theta2.getRadians();
        double omega1 = state.omega1, omega2 = state.omega2;


        double C1 = -m2 * l1 * r2 * Math.sin(theta2) * omega2;
        double C2 = -m2 * l1 * r2 * Math.sin(theta2) * (omega2 + omega1);
        double C3 = m2 * l1 * r2 * Math.sin(theta2);
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
        double theta1 = state.theta1.getRadians(), theta2 = state.theta2.getRadians();
        double G1 = (m1 * r1 + m2 * l1) * g * Math.cos(theta1) + m2 * r2 * g * Math.cos(theta1 + theta2);
        double G2  = m2 * r2 * g * Math.cos(theta1 + theta2);

        return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                G1,
                G2
        );

    }


    public ArmState getAccels(ArmState state, double inputVolt1, double inputVolt2) {
        Matrix<N2, N1> omegas = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                state.omega1,
                state.omega2
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

        Matrix<N2, N1> accels = M.inv().times(torque.minus(C.times(omegas)).minus(G));
        accels.set(0,0, MathUtil.clamp(accels.get(0,0), -25, 25));
        accels.set(1,0, MathUtil.clamp(accels.get(1,0), -25, 25));


        return new ArmState(state.theta1, state.theta2, state.omega1, state.omega2, accels.get(0,0), accels.get(1,0));
    }

    /**
     * Gets feedForward Voltages
     * @param states ArmStates values (include accels if you can)
     * @return [voltage 1, voltage v2]
     */
    public Matrix<N2, N1> feedForward(ArmState states) {
        Matrix<N2, N2> M = getDynamicMatrixM(states);
        Matrix<N2, N2> C = getDynamicMatrixC(states);
        Matrix<N2, N1> G = getDynamicMatrixG(states);

        double omega1 = states.omega1, omega2 = states.omega2;
        double alpha1 = states.accel1, alpha2 = states.accel2;

        Matrix<N2, N1> accels = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                alpha1,
                alpha2
        );
        Matrix<edu.wpi.first.math.numbers.N2, edu.wpi.first.math.numbers.N1> omegas = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                omega1,
                omega2
        );

        return K3.inv().times(
                M.times(accels).plus(C.times(omegas)).plus(G).plus(K4.times(omegas))
                //C.times(omegas)).plus(G).plus(K4.times(omegas)
        );
    }


    public void setVoltages(double volt1, double volt2) {
        m_motor1.setVoltage(volt1);
        m_motor2.setVoltage(volt2);
    }

    @Override
    public void periodic() {
        ArmState goalState = getGoalState();

        if(goalState.theta1.getDegrees() <= 50 || goalState.theta1.getDegrees() >= 130) return;
        if(getCurrentState().theta1.getDegrees() <= 50 || getCurrentState().theta1.getDegrees() >= 200) return;

        double input1 = controller1.calculate(getCurrentState().theta1.getRadians(), goalState.theta1.getRadians());
        double input2 = controller2.calculate(getCurrentState().theta2.getRadians(), goalState.theta2.getRadians());

        ArmState accels = getAccels(getCurrentState(), lastinput1, lastinput2);

        Matrix<N2, N1> ffs = feedForward(accels);
        SmartDashboard.putNumber("ff1", ffs.get(0, 0));
        SmartDashboard.putNumber("ff2", ffs.get(1,0));

        lastinput1 = MathUtil.clamp(input1  + ffs.get(0, 0), -8, 8);
        lastinput2 = MathUtil.clamp(input2 + ffs.get(1, 0), -8, 8);

        setVoltages(lastinput1, lastinput2);
    }

    public static double normalizeAngle(double radians) {
        while(radians < 0) {
            radians += Math.PI * 2;
        }
        while(radians > 2 * Math.PI) {
            radians -= Math.PI;
        }

        return radians;
    }
}
