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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.Vision;

import java.util.Optional;

public class LimeLight extends SubsystemBase implements SubsystemLogging{

  private static IntegerPublisher getpipePub;
  private static IntegerSubscriber getPipeSub;

  private static DoubleSubscriber tx;
  private static DoubleSubscriber ty;
  private static DoubleSubscriber tv;
  private static DoubleSubscriber ta;
  private static IntegerSubscriber tid;
  private static IntegerSubscriber tl;
  private static DoubleArraySubscriber camTran;
  private static DoubleArraySubscriber botpose;

  private static IntegerPublisher camModePub;
  private static IntegerSubscriber camModeSub;
  private static IntegerPublisher ledModePub;
  private static IntegerSubscriber ledModeSub;

  private static DoublePublisher distance;

  private static final long[] legalTags = new long[] {1,2,3,4,5,6,7,8};

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
  /** Creates a new LimeLight. */
  public LimeLight() {

    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");


    getPipeSub = limeLightTable.getIntegerTopic("getpipe").subscribe(0);

    tx = limeLightTable.getDoubleTopic("tx").subscribe(0); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty = limeLightTable.getDoubleTopic("ty").subscribe(0); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = limeLightTable.getDoubleTopic("tv").subscribe(0); // Whether the limelight has any valid targets (0 or 1).
    ta = limeLightTable.getDoubleTopic("ta").subscribe(0); // Target area (0% of image to 100% of image).
    tid = limeLightTable.getIntegerTopic("tid").subscribe(0);
    tl = limeLightTable.getIntegerTopic("tl").subscribe(999);
    camTran = limeLightTable.getDoubleArrayTopic("camTran").subscribe(new double[] {});
    ledModeSub = limeLightTable.getIntegerTopic("ledMode").subscribe(0); // limelight's LED state (0-3).
    camModeSub = limeLightTable.getIntegerTopic("camMode").subscribe(0); // limelight's operation mode (0-1).
    botpose = limeLightTable.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});


    ledModePub = limeLightTable.getIntegerTopic("ledMode").publish();
    camModePub = limeLightTable.getIntegerTopic("camMode").publish();
    getpipePub = limeLightTable.getIntegerTopic("getpipe").publish();
    distance = limeLightTable.getDoubleTopic("distance").publish();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTapeDistance().ifPresent((dist) -> distance.set(dist));
    updateLogging();
  }

  /**
   * Get the ID for the active pipeline
   * @return ID from 0-9
   */
  public long getActivePipeLine() {
    return getPipeSub.get();
  }

  /**
   * Horizontal offset from crosshair to target.
   * @return offset from -29.8 to 29.8 degrees.
   */
  public double getTargetOffsetX()
  {
    return tx.get(0.0);
  }

  /**
   * Vertical offset from crosshair to target.
   * @return offset from -24.85 to 24.85 degrees.
   */
  public double getTargetOffsetY()
  {
    return ty.get(0.0);
  }

  /**
   * Get whether a target is detected.
   * @return true if target is found and false if target is not found.
   */
  public boolean isTargetAvailable()
  {
    return tv.get() == 1;
  }

  /**
   * Get area of detected target.
   * @return target area from 0% to 100%.
   */
  public double getTargetArea()
  {
    return ta.get(0.0);
  }

  public Optional<Double> getTapeDistance()
  {
    double targetOffsetAngle = ty.get(-1);

    double angleToGoalRadians = Units.degreesToRadians(Vision.limeLightMountAngleDegrees + targetOffsetAngle);
    if(isTargetAvailable()) {
      return Optional.of(Vision.goalHeighInches - Vision.limeligtLensHeighInches/(Math.tan(angleToGoalRadians)));

    }
    return Optional.empty();

  }


  /**
   * Get ID of detected AprilTag
   * @return AprilTag ID from 1-8
   */
  public long getTargetID() {
    long id = tid.get(-1);
    for(long i : legalTags) {
      if(id == i) {
        return id;
      }
    }
    return -1;

  }

  /**
   * Get Pipeline latency + 11ms for Image Capture
   * @return Latency (ms)
   */
  public long getLatency() {
    return tl.get(988) + 11;
  }

  /**
   * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   * @return Transform from Camera to Target
   */
  public Transform3d getCamTransform3d() {

    double[] camTransform = camTran.get(new double[0]);
    return new Transform3d(
            new Translation3d(camTransform[0], camTransform[1], camTransform[2]),
            new Rotation3d(camTransform[3], camTransform[4], camTransform[5])
    );

  }

  /**
   * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   * @return Transform from Camera to Target
   */
  public Transform2d getCamTransform2d() {
    double[] camTransform = camTran.get(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if(0 != camTransform.length) {
      return new Transform2d(
              new Translation2d(camTransform[0], camTransform[1]), new Rotation3d(Units.degreesToRadians(camTransform[3]), Units.degreesToRadians(camTransform[4]), Units.degreesToRadians(camTransform[5])).toRotation2d());
    }
    return new Transform2d();
  }

  /**
   * Get Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
   * LimeLight has its own dict for apriltag poses
   * @return Pose3d of Robot
   */
  public Pose3d getBotPose() {
    double[] poseValues = botpose.get();
    if(poseValues.length != 0) {
      return new Pose3d(
              new Translation3d(poseValues[0], poseValues[1], poseValues[2]),
              new Rotation3d(Units.degreesToRadians(poseValues[3]), Units.degreesToRadians(poseValues[4]), Units.degreesToRadians(poseValues[5])));
    }
    return new Pose3d();
  }


  /**
   * Method to set the green light's status.
   * @param mode either pipeline, off, blink, or on.
   */
  private void setLEDMode(LEDMode mode)
  {
    ledModePub.set(mode.modeValue);
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
    camModePub.set(mode.modeValue);
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
    return ledModeSub.get(0) == LEDMode.OFF.modeValue && camModeSub.get(0) == CamMode.DRIVER.modeValue;
  }
  private boolean isModeVision()
  {
    return ledModeSub.get(0) == LEDMode.ON.modeValue && camModeSub.get(0) == CamMode.VISION.modeValue;
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

  @Override
  public void updateLogging() {
    log("Target Area", getTargetArea());
    log("Offset Y", getTargetOffsetY());
    log("Target Offset X", getTargetOffsetX());
    log("Target ID", getTargetID());
    log("Target Area", getTargetArea());
    log("Latency", getLatency());

  }

}