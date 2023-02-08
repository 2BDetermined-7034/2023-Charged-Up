// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Alert;
import frc.robot.util.BatteryTracker;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.constants.AdvantageKitConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import static frc.robot.constants.AdvantageKitConstants.RobotType.ROBOT_2023C;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  private final Alert logNoFileAlert =
          new Alert("No log path set for current robot. Data will NOT be logged.",
                  Alert.AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
          new Alert("Logging queue exceeded capacity, data will NOT be logged.",
                  Alert.AlertType.ERROR);

  public Robot() {
    super(AdvantageKitConstants.loopPeriodSecs);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();
    setUseTiming(AdvantageKitConstants.getMode() != AdvantageKitConstants.Mode.REPLAY);
    logger.recordMetadata("Robot", AdvantageKitConstants.getRobot().toString());
    logger.recordMetadata("TuningMode", Boolean.toString(AdvantageKitConstants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());

    switch (AdvantageKitConstants.getMode()) {
      case REAL:
        String folder = AdvantageKitConstants.logFolders.get(AdvantageKitConstants.getRobot());
        if (folder != null) {
          logger.addDataReceiver(new WPILOGWriter(folder));
        } else {
          logNoFileAlert.set(true);
        }
        logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }
    logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();

    // Log scheduled commands
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
            NetworkTableInstance.getDefault()
                    .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
                    .getStringArray(new String[] {}));

    // Check logging fault
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.println(String.format("*** Auto finished in %.2f secs ***",
                  Timer.getFPGATimestamp() - autoStart));
        } else {
          System.out
                  .println(String.format("*** Auto cancelled in %.2f secs ***",
                          Timer.getFPGATimestamp() - autoStart));
        }
        autoMessagePrinted = true;
      }
    }

    Threads.setCurrentThreadPriority(true, 10);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
