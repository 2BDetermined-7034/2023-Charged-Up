package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class GravityClawSubsystem extends SubsystemBase implements SubsystemLogging {
    private final DoubleSolenoid sol;

    public GravityClawSubsystem() {
        sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.GravityClaw.grabberFC, Constants.GravityClaw.grabberRC);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void setSolonoid(boolean direction) {
        if (direction) {
            sol.set(DoubleSolenoid.Value.kReverse);
        } else sol.set(DoubleSolenoid.Value.kForward);
    }

    public DoubleSolenoid.Value getSolenoid() {
        return sol.get();
    }

    public void ToggleSolonoid() {
        if (sol.get().equals(DoubleSolenoid.Value.kForward)) {
            sol.set(DoubleSolenoid.Value.kReverse);
        } else {
            sol.set(DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void periodic() {
        updateLogging();
    }
}

