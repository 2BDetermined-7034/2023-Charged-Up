package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;
import frc.robot.util.ArmState;

import java.util.function.DoubleSupplier;

import static frc.robot.constants.Constants.ArmConstants.kMaxArmOverrideSpeedDistal;
import static frc.robot.constants.Constants.ArmConstants.kMaxArmOverrideSpeedShoulder;

public class ArmOverrideIK extends ArmOverride{
    public ArmOverrideIK(Arm arm, DoubleSupplier getx1, DoubleSupplier gety1, DoubleSupplier enable) {
        super(arm, getx1, gety1, enable);
    }

    @Override
    public void execute() {
        arm.setInput(getx1.getAsDouble() * kMaxArmOverrideSpeedShoulder, gety1.getAsDouble() * kMaxArmOverrideSpeedDistal);
        arm.setGoalState(arm.getCurrentState().clear());

    }

}
