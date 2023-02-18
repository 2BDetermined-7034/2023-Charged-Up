package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Indexer extends SubsystemBase implements SubsystemLogging {
    public CANSparkMax indexerMotor;

    public Indexer() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        indexerMotor = new CANSparkMax(Constants.Intake.indexerMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void runIndexerCounterclockwise() {
        indexerMotor.setVoltage(Constants.Intake.indexerSpeed);
    }

    public void runIndexerClockwise() {
        indexerMotor.setVoltage(-Constants.Intake.indexerSpeed);
    }
}

