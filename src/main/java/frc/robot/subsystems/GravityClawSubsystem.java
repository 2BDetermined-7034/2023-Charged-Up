package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class GravityClawSubsystem extends SubsystemBase implements SubsystemLogging{
    private DoubleSolenoid sol;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this GravityClaSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static GravityClawSubsystem INSTANCE = new GravityClawSubsystem();

    /**
     * Returns the Singleton instance of this GravityClaSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code GravityClaSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static GravityClawSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this GravityClaSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public GravityClawSubsystem() {
        sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.GravityClaw.forwardChannel,Constants.GravityClaw.reverseChannel);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
    public void setSolonoid(boolean direction){
        if(direction){
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
}

