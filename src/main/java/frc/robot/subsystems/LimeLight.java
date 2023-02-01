// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

  private static NetworkTableEntry getpipe;

  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry tid;
  private static NetworkTableEntry tl;
  private static NetworkTableEntry camTran;
  private static NetworkTableEntry botpose;
  
  private static NetworkTableEntry tclass;

  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  private enum LEDMode
  {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);

    private int modeValue;
    private LEDMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }

  private enum CamMode
  {
    VISION(0),
    DRIVER(1);

    private int modeValue;
    CamMode(int modeVal)
    {
      this.modeValue = modeVal;
    }
  }
  /** Creates a new LimeLight. */
  public LimeLight() {

    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");


    getpipe = limeLightTable.getEntry("getpipe");

    tx = limeLightTable.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty = limeLightTable.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = limeLightTable.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = limeLightTable.getEntry("ta"); // Target area (0% of image to 100% of image).
    tid = limeLightTable.getEntry("tid");
    tl = limeLightTable.getEntry("tl");
    camTran = limeLightTable.getEntry("camTran");
    ledMode = limeLightTable.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = limeLightTable.getEntry("camMode"); // limelight's operation mode (0-1).
    botpose = limeLightTable.getEntry("botpose");

    tclass = limeLightTable.getEntry("tclass");

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get the ID for the active pipeline
   * @return ID from 0-9
   */
  public long getActivePipeLine() {
    return getpipe.getInteger(0);
  }

  /**
   * Horizontal offset from crosshair to target.
   * @return offset from -29.8 to 29.8 degrees.
   */
  public double getTargetOffsetX()
  {
    return tx.getDouble(0.0);
  }

  /**
   * Vertical offset from crosshair to target.
   * @return offset from -24.85 to 24.85 degrees.
   */
  public double getTargetOffsetY()
  {
    return ty.getDouble(0.0);
  }

  /**
   * Get whether or not a target is detected.
   * @return true if target is found and false if target is not found.
   */
  public boolean isTargetAvailable()
  {
    return tv.getNumber(0).intValue() == 1;
  }

  /**
   * Get area of detected target.                                                                                                                        
   * @return target area from 0% to 100%.
   */
  public double getTargetArea()
  {
    return ta.getDouble(0.0);
  }
  
  /**
   * Get ID of detected AprilTag
   * @return AprilTag Id from 1-8
   */
  public long getTargetID() {
    return tid.getInteger(0);
  }

  /**
   * Get Pipeline latency + 11ms for Image Capture
   * @return Latency (ms)
   */
  public long getLatency() {
    return tl.getInteger(988) + 11;
  }

  /**
   * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   * @return Transform from Camera to Target
   */
  public Transform3d getCamTransform3d() {

    Number[] camTransform = camTran.getNumberArray(new Number[6]);
    return new Transform3d(
      new Translation3d(camTransform[0].doubleValue(), camTransform[1].doubleValue(), camTransform[2].doubleValue()),
      new Rotation3d(camTransform[5].doubleValue(), camTransform[3].doubleValue(), camTransform[4].doubleValue())
    );

  }

  /**
   * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   * @return Transform from Camera to Target
   */
  public Transform2d getCamTransform2d() {
    Number[] camTransform = camTran.getDoubleArray(new Double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if(camTransform.length != 0) {
    return new Transform2d(
      new Translation2d(camTransform[0].doubleValue(), camTransform[1].doubleValue()),
      new Rotation3d(camTransform[5].doubleValue(), camTransform[3].doubleValue(), camTransform[4].doubleValue()).toRotation2d()
    );
    }
    return new Transform2d();
  }

  /**
   * Get Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
   * LimeLight has it's own dict for apriltag poses
   * @return Pose3d of Robot 
   */
  public Pose3d getBotPose() {
    Double[] poseVals = botpose.getDoubleArray(new Double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if(poseVals.length != 0) {
    return new Pose3d(
      new Translation3d(poseVals[0], poseVals[1], poseVals[2]),
      new Rotation3d(poseVals[5], poseVals[3], poseVals[4])
    );
    }
    return new Pose3d();
  }

  /**
   * Gets the Label for the primary detected object
   * @return Class of the detected object
   */
  public Class<? extends NetworkTableEntry> getDetectorClass() {
    return tclass.getClass();
  }

  /**
   * Method to set the green light's status.
   * @param mode either pipeline, off, blink, or on.
   */
  private void setLEDMode(LEDMode mode)
  {
    ledMode.setNumber(mode.modeValue);
  }

  /**
   * Methods for external classes to change green light's status.
   */
  public void turnOnLED()
  {
    this.setLEDMode(LEDMode.ON);
  }                 
  public void turnOffLED()
  {
    this.setLEDMode(LEDMode.OFF);
  }
  public void blinkLED()
  {
    this.setLEDMode(LEDMode.BLINK);
  }

  /**
   * Method to set camera mode.
   * @param mode either driver or vision mode.
   */
  private void setCamMode(CamMode mode)
  {
    camMode.setNumber(mode.modeValue);
  }

  /**
   * Method to set video feed in driver mode.
   * Turns off green light and switches camera mode to driver.
   */
  public void setModeDriver()
  {
    this.setLEDMode(LEDMode.OFF);
    this.setCamMode(CamMode.DRIVER);
  }

  /**
   * Method to set video feed in vision mode.
   * Turns on green light and switches camera mode to vision.
   */
  public void setModeVision()
  {
    this.setLEDMode(LEDMode.ON);
    this.setCamMode(CamMode.VISION);
  }

  /**
   * Methods to tell whether the limelight is in driver or vision mode.
   * Driver mode will consist of the LEDs being off and the camera being in color.
   * Vision mode will consist of the LEDs being on and the camera being in black and white.
   */
  private boolean isModeDriver()
  {
    return ledMode.getDouble(0.0) == LEDMode.OFF.modeValue && camMode.getDouble(0.0) == CamMode.DRIVER.modeValue;
  }
  private boolean isModeVision()
  {
    return ledMode.getDouble(0.0) == LEDMode.ON.modeValue && camMode.getDouble(0.0) == CamMode.VISION.modeValue;
  }
  
  /**
   * Method to toggle the type of video feed.
   */
  public void toggleMode()
  {
    if (this.isModeDriver())
    {
      this.setModeVision();
    }
    else if (this.isModeVision())
    {
      this.setModeDriver();
    }
    else
    {
      this.blinkLED();
    }
  }
}
