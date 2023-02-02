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
import edu.wpi.first.networktables.*;
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
    private DoublePublisher ffOutput;
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Arm");


    private final ProfiledPIDController controller2;
    private final ArmFeedforward armFeedforward;

    /** Creates a new Arm. */
    public Arm() {
        controller2 = new ProfiledPIDController(7, 0, 0, new TrapezoidProfile.Constraints(15, 30));
        armFeedforward = new ArmFeedforward(0.4, 0.25, 0.19, 0.01);

        controller2.setIntegratorRange(-5, 5);

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
        m_motor2Encoder.setPosition(Units.degreesToRadians(180));
        goalState = new ArmState(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(180), 0, 0, 0, 0);
        last_velocity1 = 0;
        last_velocity2 = 0;

        configureDashBoard();
    }

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

        ffOutput = networkTable.getDoubleTopic("Feed Forward").publish();
    }

    public void updateDashBoard() {
        currentTheta1.set(getCurrentState().theta1.getDegrees());
        currentTheta2.set(getCurrentState().theta2.getDegrees());

        omega1.set(getCurrentState().omega1);
        omega2.set(getCurrentState().omega2);

        alpha1.set(getCurrentState().accel1);
        alpha2.set(getCurrentState().accel2);

        targetTheta1.set(getGoalState().theta1.getDegrees());
        targetTheta2.set(getGoalState().theta2.getDegrees());

        error2.set(Math.toDegrees(controller2.getPositionError()));

        appliedOutput1.set(m_motor1.getAppliedOutput());
        appliedOutput2.set(m_motor2.getAppliedOutput());

        ffOutput.set(armFeedforward.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity, 0));

    }
    public ArmState getGoalState() {
        return goalState;
    }
    public void setGoalState(ArmState state) {
        goalState = state;
    }
    public ArmState getCurrentState() {
        Rotation2d theta1 = Rotation2d.fromRadians(normalizeAngle(m_motor1Encoder.getPosition()));
        Rotation2d theta2 = Rotation2d.fromRadians(normalizeAngle(m_motor2Encoder.getPosition()));

        double omega1 = m_motor1Encoder.getVelocity();
        double omega2 = m_motor2Encoder.getVelocity();
        double firstAcceleration = (m_motor1Encoder.getVelocity() - last_velocity1) / (0.02);
        double secondAcceleration = (m_motor2Encoder.getVelocity() - last_velocity2) / (0.02);
        last_velocity1 = m_motor1Encoder.getVelocity();
        last_velocity2 = m_motor2Encoder.getVelocity();

        return new ArmState(theta1, theta2, omega1, omega2, firstAcceleration, secondAcceleration);
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





    public void setVoltages(double volt1, double volt2) {
        m_motor1.setVoltage(volt1);
        m_motor2.setVoltage(volt2);
    }

    @Override
    public void periodic() {
        ArmState goalState = getGoalState();

        double input2 = controller2.calculate(getCurrentState().theta2.getRadians(), goalState.theta2.getRadians());
        double betterFeedForward = armFeedforward.calculate(controller2.getSetpoint().position, controller2.getSetpoint().velocity, 0);

        setVoltages(0, MathUtil.clamp(input2 - betterFeedForward, -12, 12));

        updateDashBoard();
    }

    public static double normalizeAngle(double radians) {
        /*
        while(radians < 0) {
            radians += Math.PI * 2;
        }
        while(radians > 2 * Math.PI) {
            radians -= Math.PI;
        }

         */

        return radians;
    }
}
