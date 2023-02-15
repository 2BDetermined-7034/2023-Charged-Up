package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.IMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {

    public CANSparkMax motor1;
    public CANSparkMax motor2;

   public MotorControllerGroup motorControllerGroup;


    public Intake() {
        motor1 = new CANSparkMax(Constants.Intake.intakeMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.Intake.intakeMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);

        motorControllerGroup = new MotorControllerGroup(motor1, motor2);

    }

    public void runIntakeForward () {
        motorControllerGroup.setVoltage(Constants.Intake.intakeSpeed);
    }

    public void runIntakeBackwards () {
        motorControllerGroup.setVoltage(-Constants.Intake.intakeSpeed);
    }


}

