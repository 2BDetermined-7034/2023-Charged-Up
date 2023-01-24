// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTableEntry;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

/** Vision hardware implementation for a Limelight. */
public class VisionIOLimelight implements frc.robot.subsystems.Vision.VisionIO {
  private double[] botpose = new double[0];
  private double  tv = 0.0;
  private double ledMode = 0;
  private double camMode = 0;
  private double tl = 999;
  private double getpipe = 0;
  private double tid = -1;


  private final NetworkTableEntry validEntry =
      getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry robotPosition = 
      getDefault().getTable("limelight").getEntry("botpose");
  private final NetworkTableEntry ledEntry =
          getDefault().getTable("limelight").getEntry("ledMode");
  private final NetworkTableEntry pipelineEntry =
          getDefault().getTable("limelight").getEntry("getpipe");
  private final NetworkTableEntry latency =
          getDefault().getTable("limelight").getEntry("tl");
  private final NetworkTableEntry cameraMode =
          getDefault().getTable("limelight").getEntry("camMode");
  private final NetworkTableEntry targetID =
          getDefault().getTable("limelight").getEntry("tid");

  public VisionIOLimelight() {
    synchronized (VisionIOLimelight.this) {
      tv = validEntry.getDouble(tv);
      ledMode = ledEntry.getDouble(ledMode);
      tl = latency.getDouble(tl);
      camMode = cameraMode.getDouble(camMode);
      botpose = robotPosition.getDoubleArray(botpose);
      getpipe = pipelineEntry.getDouble(getpipe);
      tid = targetID.getDouble(-1);
    }
  }


  public synchronized void updateInputs(VisionIO.VisionIOInputs inputs) {
    inputs.tv = tv;
    inputs.botpose = botpose;
    inputs.ledMode = ledMode;
    inputs.tl = tl;
    inputs.getpipe = getpipe;
    inputs.tid = tid;
  }


  public void setLeds(int mode) {
    ledEntry.setDouble(mode);
  }


  public void setCamMode(int mode) {
    cameraMode.setNumber(mode);
  }


  public void setPipeline(int pipeline) {
    pipelineEntry.setNumber(pipeline);
  }
}