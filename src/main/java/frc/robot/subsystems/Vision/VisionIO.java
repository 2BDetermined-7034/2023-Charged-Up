// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  public static class VisionIOInputs implements LoggableInputs {
    public double tv = 0.0, ledMode = 0.0, tl = 999, getpipe = 0, camMode = 0, tid = 0;
    public double[] botpose = new double[] {};



    public void toLog(LogTable table) {
      table.put("tv", tv);
      table.put("botpose", botpose);
      table.put("ledMode", ledMode);
      table.put("tl", tl);
      table.put("getpipe", getpipe);
      table.put("camMode", camMode);
      table.put("tid", tid);
    }

    public void fromLog(LogTable table) {
      tv = table.getDouble("tv", tv);
      botpose = table.getDoubleArray("botpose", botpose);
      ledMode = table.getDouble("ledMode", ledMode);
      tl = table.getDouble("tl", tl);
      getpipe = table.getDouble("getpipe", getpipe);
      camMode = table.getDouble("camMode", camMode);
      tid = table.getDouble("tid", tid);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(int mode) {}

  public default void setCamMode(int mode) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}