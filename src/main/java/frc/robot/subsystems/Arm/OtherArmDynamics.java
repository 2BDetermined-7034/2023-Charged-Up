package frc.robot.subsystems.Arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class OtherArmDynamics {
    ArmConfig.JointConfig elbow;
    ArmConfig.JointConfig shoulder;
    private final double g = -9.81;



    public OtherArmDynamics(ArmConfig config) {
        this.elbow = config.elbow();
        this.shoulder = config.shoulder();
    }
    public Matrix<N2, N1> feedForward(ArmState state) {
        return B().inv().times(
                M(state.getPositionVector())
                        .times(state.getAlphaVector())
                        .plus(C(state.getPositionVector(), state.getOmegaVector()).times(state.getOmegaVector()))
                        .plus(Tg(state.getPositionVector()))
                        .plus(Kb().times(state.getOmegaVector())));
    }

    private Matrix<N2, N2> M(Vector<N2> position) {
        var M = new Matrix<>(N2.instance, N2.instance);
        M.set(
                0,
                0,
                elbow.mass() * shoulder.length() * shoulder.length() + shoulder.mass() * shoulder.cgRadius() * shoulder.cgRadius() + shoulder.moi());
        M.set(
                1,
                0,
                shoulder.length() * elbow.mass() * elbow.cgRadius() * Math.cos(position.get(0,0) - position.get(1,0)));
        M.set(
                0,
                1,
                shoulder.length() * elbow.mass() * elbow.cgRadius() * Math.cos(position.get(0,0) - position.get(1,0)));
        M.set(
                1,
                1,
                elbow.mass() * elbow.cgRadius() * elbow.cgRadius() + elbow.moi());
        return M;
    }

    private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        var C = new Matrix<>(N2.instance, N2.instance);
        C.set(
                1,
                0,
                shoulder.length() * elbow.mass() * elbow.cgRadius() * Math.sin(position.get(1,0) - position.get(0,0)) * velocity.get(0,0));
        C.set(
                0,
                1,
                shoulder.length() * elbow.mass() * elbow.cgRadius() * Math.sin(position.get(0,0) - position.get(1,0)) * velocity.get(1,0));
        return C;
    }

    private Matrix<N2, N1> Tg(Vector<N2> position) {
        var Tg = new Matrix<>(N2.instance, N1.instance);
        Tg.set(
                0,
                0,
                g * Math.cos(position.get(0, 0)) * (shoulder.length() * elbow.mass() + shoulder.mass() * shoulder.cgRadius())
        );
        Tg.set(
                1,
                0,
                g * elbow.mass() * elbow.cgRadius() * Math.cos(position.get(1, 0)));
        return Tg;
    }

    private Matrix<N2, N2> B() {
        Matrix<N2, N2> B = new Matrix<>(Nat.N2(), Nat.N2());
        B.set(0,0, shoulder.motor().reduction() * (shoulder.motor().physics().stallTorqueNewtonMeters/shoulder.motor().physics().stallCurrentAmps) / shoulder.cgRadius());
        B.set(1,1, elbow.motor().reduction() * (elbow.motor().physics().stallTorqueNewtonMeters/elbow.motor().physics().stallCurrentAmps) / elbow.cgRadius());
        return B;
    }

    private Matrix<N2, N2> Kb() {
        Matrix<N2, N2> Kb = new Matrix<>(Nat.N2(), Nat.N2());
        Kb.set(0,0, shoulder.motor().reduction() * shoulder.motor().reduction() *(shoulder.motor().physics().stallTorqueNewtonMeters/shoulder.motor().physics().stallCurrentAmps) / (shoulder.motor().physics().freeSpeedRadPerSec * shoulder.motor().physics().stallCurrentAmps));
        Kb.set(1,1, elbow.motor().reduction() * elbow.motor().reduction() *(elbow.motor().physics().stallTorqueNewtonMeters/shoulder.motor().physics().stallCurrentAmps) / (elbow.motor().physics().freeSpeedRadPerSec * elbow.motor().physics().stallCurrentAmps));
        return Kb;
    }

}
