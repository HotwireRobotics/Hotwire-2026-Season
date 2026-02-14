package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Real Limelight vision IO: reads bot pose estimate from NT and sends orientation to the camera.
 */
public class VisionIOLimelight implements VisionIO {

  private static final double MAX_AVG_TAG_DIST = 3.0;

  private final String limelightName;

  public VisionIOLimelight(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d robotPoseForCamera) {
    LimelightHelpers.SetIMUMode(limelightName, 2);
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    LimelightHelpers.SetRobotOrientation(
        limelightName, robotPoseForCamera.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if (estimate != null && estimate.tagCount > 0 && estimate.avgTagDist < MAX_AVG_TAG_DIST) {
      inputs.valid = true;
      inputs.pose = estimate.pose;
      inputs.timestampSeconds = estimate.timestampSeconds;
      inputs.tagCount = estimate.tagCount;
      inputs.avgTagDist = estimate.avgTagDist;
    } else {
      inputs.valid = false;
      inputs.pose = new Pose2d();
      inputs.timestampSeconds = 0.0;
      inputs.tagCount = 0;
      inputs.avgTagDist = 0.0;
    }
  }
}
