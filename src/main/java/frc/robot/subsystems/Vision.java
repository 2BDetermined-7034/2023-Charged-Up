// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;



public class Vision extends SubsystemBase {

  private RobotPoseEstimator poseEstimator;
  private PoseStrategy estimatorStrategy = PoseStrategy.LOWEST_AMBIGUITY;


  public static final ArrayList<AprilTag> aprilTags = new ArrayList<>();
    static 
    {
      for(int i : FieldConstants.aprilTags.keySet()) {
        aprilTags.add(new AprilTag(i, FieldConstants.aprilTags.get(i)));
      }
    }


  /** Creates a new Vision. */
  public Vision() {
    

    

    AprilTagFieldLayout atfl = new AprilTagFieldLayout(aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth);

    poseEstimator =
                new RobotPoseEstimator(atfl, estimatorStrategy, Constants.VisionConstants.camList);

    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Method to calculate global robot Position, Rotation
   * Returns RobotPose2d, latency (ms)
   * @param prevEstimatedRobotPose
   * @return
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(
                result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }

    
  }


  /**
   * Influence the PoseEstimator with a reference
   * @param referencePose
   */
  public void setReferencePose(Pose2d referencePose) {
      poseEstimator.setReferencePose(referencePose);
  }

  /**
   * 
   * Sets the Strategy used by the PoseEstimator
   * 
   *CLOSEST_TO_CAMERA_HEIGHT
   *Choose the Pose which is closest to the camera height.
   *CLOSEST_TO_REFERENCE_POSE
   *Choose the Pose which is closest to the pose from setReferencePose().
   *CLOSEST_TO_LAST_POSE
   *Choose the Pose which is closest to the last pose calculated.
   *AVERAGE_BEST_TARGETS
   *Choose the Pose which is the average of all the poses from each tag.
   * 
   * @param strat
   */
  public void setPoseEstimatorStrategy(PoseStrategy strat) {
    poseEstimator.setStrategy(strat);
  }
}
