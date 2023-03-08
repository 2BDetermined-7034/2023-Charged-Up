package frc.robot.subsystems.Arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N1;

import static frc.robot.constants.Constants.ArmConstants.*;


public class ArmState {

    private Rotation2d theta1;
    private Rotation2d theta2;
    private double omega1;
    private double omega2;
    private double accel1;
    private double accel2;

    public ArmState(Rotation2d theta1, Rotation2d theta2, double omega1, double omega2, double accel1, double accel2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.omega1 = omega1;
        this.omega2 = omega2;
        this.accel1 = accel1;
        this.accel2 = accel2;
    }
    public ArmState(Rotation2d theta1, Rotation2d theta2, double omega1, double omega2) {
        this(theta1, theta2, omega1, omega2, 0, 0);
    }

    public ArmState(Rotation2d theta1, Rotation2d theta2) {
        this(theta1, theta2, 0, 0);
    }

    public ArmState(double theta1, double theta2) {this(Rotation2d.fromRadians(theta1), Rotation2d.fromRadians(theta2), 0, 0);}

    public ArmState(double theta1, double theta2, double omega1, double omega2, double alpha1, double alpha2) {
        this(Rotation2d.fromRadians(theta1), Rotation2d.fromRadians(theta2), omega1, omega2, alpha1, alpha2);
    }

    public ArmState(Matrix<N4, N1> mat) {
        this(mat.get(0,0), mat.get(1,0), mat.get(2,0), mat.get(3,0), 0, 0);

    }

    public ArmState(double degreesToRadians, double degreesToRadians1, double omega1, double omega2) {
        this(degreesToRadians, degreesToRadians1, omega1, omega2, 0, 0);
    }

    /**
     * Checks if an inverse-kinematics solution exists for the given point in space
     * @param x position in space of end effector
     * @param y position in space of end effector
     * @return whether the solution exists
     */
    private boolean solutionExists(double x, double y) {
        double sqrt = Math.sqrt(x * x + y * y);
        return !(l1 + l2 < sqrt || Math.abs(l1 - l2) > sqrt);
    }

//    public static Matrix<edu.wpi.first.math.numbers.N2, N1> inverseKinematics(double x, double y) {
//        double theta2 = Math.acos(x * x + y * y - )
//    }

    /** Getter Methods */
    public double getTheta1() {
        return this.theta1.getRadians();
    }

    public double getTheta2() {
        return this.theta2.getRadians();
    }

    public double getOmega1() {
        return this.omega1;
    }
    public double getOmega2() {
        return this.omega2;
    }
    public double getAlpha1() {
        return this.accel1;
    }
    public double getAlpha2() {
        return this.accel2;
    }
    public Matrix<N4, N1> getStateMatrixDot() {
        return VecBuilder.fill(omega1, omega2, accel1, accel2);
    }
    public Matrix<N4, N1> getStateMatrix4() {
        return VecBuilder.fill(getTheta1(), getTheta2(), omega1, omega2);
    }
    public ArmState clear() {
        this.omega1 = 0;
        this.omega2 = 0;
        this.accel1 = 0;
        this.accel2 = 0;
        return this;
    }

    public ArmState incrementDegrees(double theta1, double theta2) {
        this.theta1 = Rotation2d.fromDegrees(this.theta1.getDegrees() + theta1);
        this.theta2 = Rotation2d.fromDegrees(this.theta2.getDegrees() + theta2);
        return this;

    }

    public Vector<N2> getPositionVector() {
        return VecBuilder.fill(getTheta1(), getTheta2());
    }
    public Vector<N2> getOmegaVector() {
        return VecBuilder.fill(omega1, omega2);
    }



}