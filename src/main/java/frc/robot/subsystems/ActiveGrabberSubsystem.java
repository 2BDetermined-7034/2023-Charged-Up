package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ActiveGrabberSubsystem extends SubsystemBase {

    WPI_TalonSRX mmFunnyMotor;

    public ActiveGrabberSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        mmFunnyMotor = new WPI_TalonSRX(Constants.GravityClaw.activeGrabberMotor);

    }


    public void runGrabberMotor(double volts) {
        mmFunnyMotor.set(volts);
    }

    public void stopGrabberMotor() {
        mmFunnyMotor.set(0);
    }



}

