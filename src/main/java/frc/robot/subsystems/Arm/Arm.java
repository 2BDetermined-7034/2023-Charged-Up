// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.Constants.ArmConstants.*;

//Check out this https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
//Copied from here


public class Arm extends SubsystemBase {

  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;

  /** Creates a new Arm. */
  public Arm() {
    m_motor1 = new CANSparkMax(motor1ID, CANSparkMaxLowLevel.MotorType.kBrushed);
    m_motor2 = new CANSparkMax(motor2ID, CANSparkMaxLowLevel.MotorType.kBrushed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Made for convenience
   */
  public static class ArmState {
    public double theta1;
    public double theta2;
    public double omega1;
    public double omega2;
    public double inputError1;
    public double inputError2;

    ArmState(double[] states) {
      this.theta1 = states[0];
      this.theta2 = states[1];
      this.omega1 = states[2];
      this.omega2 = states[3];
      if(states.length == 6){
        this.inputError1 = states[4];
        this.inputError2 = states[5];
      }

    }

    public ArmState(Matrix<N2, N1> thetas) {
      this.theta1 = thetas.get(1, 1);
      this.theta2 = thetas.get(1, 2);
      this.omega2 = 0;
      this.omega1 = 0;
    }
  }

  /**
   *
   * @param state
   * @return
   */
  public Matrix<N1, N2> joint2fwdKinematics(ArmState state) {

    Matrix<N1, N2> joint2 = joint1fwdKinematics(state);

    return joint2.plus(
            new MatBuilder<>(Nat.N1(), Nat.N2()).fill(
                    l2 * Math.cos(state.theta1 + state.theta2),
                    l2 * Math.sin(state.theta1 + state.theta2)
            )
    );

  }

  /**
   * Inverts kinematics for a target position xy
   * @param xy double[] {x, y}
   * @return
   */
  public static Matrix<N2, N1> inverseKinematics(double[] xy) {
    double x = xy[0];
    double y = xy[1];
    double theta2 = Math.acos((x * x + y * y - (l1 * l1 + l2 * l2)) / (2 * l1 * l2));

    double theta1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(theta2), l1 + l2 * Math.cos(theta2));
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(theta1, theta2);
  }

  public Matrix<edu.wpi.first.math.numbers.N1, edu.wpi.first.math.numbers.N2> joint1fwdKinematics(ArmState state) {
    return new MatBuilder<>(Nat.N1(), Nat.N2()).fill(
            l1 * Math.cos(state.theta1),
            l1 * Math.sin(state.theta1)
    );
  }

  //I know this is stupid, but I want to know if this will work

  /**
   * gets the intertia matrix
   * @param state
   * @return
   */
  private Matrix<N2, N2> getDynamicMatrixM(ArmState state) {

    //Encoder stuff
    double theta2 = state.theta2;


    double c2 = Math.cos(theta2);


    double hM = l1 * r2 * c2;

    //Inertia Matrix
    return
            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(r1 * r1, 0, 0, 0).times(m1)
                    .plus(
                            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                                    l1 * l1 + r2 * r2 + 2 * hM,
                                    r2 * r2 + hM,
                                    r2 * r2 + hM,
                                    r2 * r2
                            ).times(m2)
                    )
                    .plus(
                            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                                    1,
                                    0,
                                    0,
                                    0
                            ).times(I1)
                    )
                    .plus(
                            new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
                                    1,
                                    1,
                                    1,
                                    1
                            ).times(I2)
                    );


  }

  /**
   * Represents centrifugal and coriolis terms
   * @param state
   * @return
   */
  public Matrix<N2, N2> getDynamicMatrixC(ArmState state) {

    double theta2 = state.theta2;
    double omega1 = state.omega1, omega2 = state.omega2;




    double hC = -m2 * l1 * r2 * Math.sin(theta2);

    return new MatBuilder<>(Nat.N2(), Nat.N2()).fill(
            hC * omega2, hC * omega1 + hC * omega2,
            -hC * omega1, 0
    );

  }

  /**
   * Torque applied by gravity
   * @param state
   * @return
   */
  public Matrix<N2, N1> getDynamicMatrixG(ArmState state) {
    double theta1 = state.theta1, theta2 = state.theta2;



    return new MatBuilder<>(Nat.N1(), Nat.N2()).fill(
                    m1 * r1 + m2 * l1,
                    0
            ).times(g * Math.cos(theta1))
            .transpose()
            .plus(
                    new MatBuilder<>(Nat.N1(), Nat.N2()).fill(
                                    m2 * r2,
                                    m2 * r2
                            ).times(g * Math.cos(theta1 + theta2))
                            .transpose()
            );

  }

  /**
   * Gets feedForward Voltages
   * @param states ArmStates values
   * @param accels theta double dot
   * @return [voltage 1, voltage v2]
   */
  public Matrix<N2, N1> feedForward(ArmState states, Matrix<N2, N1> accels) {
    Matrix<N2, N2> M = getDynamicMatrixM(states);
    Matrix<N2, N2> C = getDynamicMatrixC(states);
    Matrix<N2, N1> G = getDynamicMatrixG(states);


    Matrix<N2, N1> omegas = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
        states.omega1,
            states.omega2
    );

    return K3.inv().times(
            M.times(accels).plus(C.times(omegas)).plus(G).plus(K4.times(omegas))
    );
  }
  /**
   * Gets feedForward Voltages wrapper
   * @param states ArmStates values
   * @return
   */
  public Matrix<N2, N1> feedForward(ArmState states) {return this.feedForward(states, new Matrix<>(Nat.N2(), Nat.N1()));}

  public void setVoltages(double volt1, double volt2) {
    m_motor1.setVoltage(volt1);
    m_motor2.setVoltage(volt2);
  }

  public double[] getThetaValues() {
    return null;
  }

  public ArmState getState() {
    return new ArmState(new double[]{
            getThetaValues()[0],
            getThetaValues()[1],
            m_motor1.getEncoder().getVelocity(),
            m_motor2.getEncoder().getVelocity()
    }
    );
  }
}
