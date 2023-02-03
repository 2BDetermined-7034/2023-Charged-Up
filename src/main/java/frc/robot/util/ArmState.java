package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.Arm.Arm;

import static frc.robot.constants.Constants.ArmConstants.l1;
import static frc.robot.constants.Constants.ArmConstants.l2;


public class ArmState {

    public Rotation2d theta1, theta2;
    public double omega1, omega2, accel1, accel2;


    public ArmState(Rotation2d theta1, Rotation2d theta2, double vtheta1, double vtheta2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.omega1 = vtheta1;
        this.omega2 = vtheta2;
    }


    public ArmState(Matrix<N2, N1> thetas) {
        theta1 = Rotation2d.fromRadians(thetas.get(0, 0));
        theta2 = Rotation2d.fromRadians(thetas.get(1, 0));
        omega2 = 0;
        omega1 = 0;
    }

    public ArmState(double omega1, double omega2, double accel1, double accel2) {
        this.omega1 = omega1;
        this.omega2 = omega2;
        this.accel1 = accel1;
        this.accel2 = accel2;
    }

    public ArmState(Rotation2d theta1, Rotation2d theta2, double omega1, double omega2, double accel1, double accel2) {
        this(theta1, theta2, omega1, omega2);
        this.accel1 = accel1;
        this.accel2 = accel2;
    }

    public ArmState(Rotation2d theta1, Rotation2d theta2) {
        this(theta1, theta2, 0, 0);
    }

    public ArmState(double x, double y, boolean invert, ArmState currentState) {

        Rotation2d theta1 = new Rotation2d();
        Rotation2d theta2 = new Rotation2d();

        if (Arm.solutionExists(x, y)) {
            theta2 = Rotation2d.fromRadians(Math.acos((x * x + y * y - (l1 * l1 + l2 * l2)) / (2 * l1 * l2)));
            theta1 = Rotation2d.fromRadians(Math.atan2(y, x) + (invert ? -1 : 1) * Math.atan2(l2 * Math.sin(theta2.getRadians()), l1 + l2 * Math.cos(theta2.getRadians())));
            this.theta1 = theta1;
            this.theta2 = theta2;
            return;
        }

        this.theta1 = currentState.theta1;
        this.theta2 = currentState.theta2;
    }

}