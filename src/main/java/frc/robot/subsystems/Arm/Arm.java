// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemLogging;
import frc.robot.util.ArmState;

import static frc.robot.constants.Constants.ArmConstants.*;

public class Arm extends SubsystemBase implements SubsystemLogging {
    private final CANSparkMax m_motor1, m_motor2;
    private final RelativeEncoder m_motor1Encoder, m_motor2Encoder;
    private final ProfiledPIDController controller1, controller2;
    private final ArmFeedforward armFeedForward1, armFeedForward2;
    private ArmState goalState;
    private double input1, input2;
    private double last_velocity1, last_velocity2;
    private boolean isOpenLoop;

    /**
     * Creates a new Arm.
     */
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
        m_motor2Encoder = m_motor2.getEncoder();

        m_motor1Encoder.setPositionConversionFactor(S1);
        m_motor2Encoder.setPositionConversionFactor(S2);

        m_motor1Encoder.setVelocityConversionFactor(S1 / 60);
        m_motor2Encoder.setVelocityConversionFactor(S2 / 60);

        m_motor1Encoder.setPosition(Units.degreesToRadians(90));
        m_motor2Encoder.setPosition(Units.degreesToRadians(270));
        goalState = new ArmState(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(270), 0, 0, 0, 0);
        last_velocity1 = 0;
        last_velocity2 = 0;

        isOpenLoop = false;
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
        double firstAcceleration = (m_motor1Encoder.getVelocity() - last_velocity1) / (0.02);
        double secondAcceleration = (m_motor2Encoder.getVelocity() - last_velocity2) / (0.02);
        last_velocity1 = m_motor1Encoder.getVelocity();
        last_velocity2 = m_motor2Encoder.getVelocity();

        return new ArmState(theta1, theta2, omega1, omega2, firstAcceleration, secondAcceleration);
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
        updateLogging();
    }


}
