// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Alert logNoFileAlert =
          new Alert("No log path set for current robot. Data will NOT be logged.", Alert.AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
          new Alert("Logging queue exceeded capacity, data will NOT be logged.", Alert.AlertType.ERROR);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("Robot", AdvantageKitConstants.getRobot().toString());
    logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
    logger.recordMetadata("TuningMode", Boolean.toString(AdvantageKitConstants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());


    // Set up data receivers & replay source
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
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    setUseTiming(AdvantageKitConstants.getMode() != AdvantageKitConstants.Mode.REPLAY);
    logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
            (Command command, Boolean active) -> {
              String name = command.getName();
              int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
              commandCounts.put(name, count);
              Logger.getInstance()
                      .recordOutput(
                              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
              Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
            };
    CommandScheduler.getInstance()
            .onCommandInitialize(
                    (Command command) -> {
                      logCommandFunction.accept(command, true);
                    });
    CommandScheduler.getInstance()
            .onCommandFinish(
                    (Command command) -> {
                      logCommandFunction.accept(command, false);
                    });
    CommandScheduler.getInstance()
            .onCommandInterrupt(
                    (Command command) -> {
                      logCommandFunction.accept(command, false);
                    });

    // Instantiate RobotContainer
    m_robotContainer = new RobotContainer();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
