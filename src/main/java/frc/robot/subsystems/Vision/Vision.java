// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {

  private final VisionIO io;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private enum LEDMode
  {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private final int modeValue;
    LEDMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }

  private enum CamMode
  {
    VISION(0),
    DRIVER(1);

    private final int modeValue;
    CamMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public boolean isTargetAvailable() {
    return inputs.tv == 1.0;
  }

  public int getTargetID() {
    return (int) inputs.tid;
  }

  public double getLatency() {
    return inputs.tl;
  }

  public Pose3d getBotPose() {
    return new Pose3d(
            new Translation3d(inputs.botpose[0], inputs.botpose[1], inputs.botpose[2]),
            new Rotation3d(Units.degreesToRadians(inputs.botpose[3]), Units.degreesToRadians(inputs.botpose[4]), Units.degreesToRadians(inputs.botpose[5]))
    );
  }

  public void setLeds(LEDMode mode) {
    io.setLeds(mode.modeValue);
  }

  public void setCamera(CamMode mode) {
    io.setCamMode(mode.modeValue);
  }

  public void setModeDriver() {
    setCamera(CamMode.DRIVER);
    setLeds(LEDMode.OFF);
  }

  public void setModeVision()
  {
    this.setLeds(LEDMode.ON);
    this.setCamera(CamMode.VISION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }
}
