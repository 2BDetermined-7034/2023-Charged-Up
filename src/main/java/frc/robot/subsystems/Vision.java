// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;



public class Vision extends SubsystemBase {

  private List<PhotonCamera> cameras;
  private RobotPoseEstimator poseEstimator;


  public static final ArrayList<AprilTag> aprilTags = new ArrayList<>();
    static 
    {
      for(int i : FieldConstants.aprilTags.keySet()) {
        aprilTags.add(new AprilTag(i, FieldConstants.aprilTags.get(i)));
      }
    }


  /** Creates a new Vision. */
  public Vision() {
    

    

    AprilTagFieldLayout atfs = new AprilTagFieldLayout(aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth);

    for



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
