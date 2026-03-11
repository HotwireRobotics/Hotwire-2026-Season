package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Real Limelight-backed IO implementation for pose and targeting logs. */
public class VisionIOLimelight implements VisionIO {
  private final String[] cameraNames;
  private double lastHeadingDeg = 0.0;

  /** Uses the provided camera names in fixed index order for logging. */
  public VisionIOLimelight(String[] cameraNames) {
    this.cameraNames = cameraNames.clone();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    final int count = cameraNames.length;
    inputs.cameraCount = count;
    inputs.connected = new boolean[count];
    inputs.hasTarget = new boolean[count];
    inputs.txDegrees = new double[count];
    inputs.tyDegrees = new double[count];
    inputs.taPercent = new double[count];
    inputs.fiducialId = new double[count];
    inputs.pipelineLatencyMs = new double[count];
    inputs.captureLatencyMs = new double[count];
    inputs.tagCount = new double[count];
    inputs.avgTagDistMeters = new double[count];
    inputs.poseX = new double[count];
    inputs.poseY = new double[count];
    inputs.poseThetaRad = new double[count];
    inputs.timestampSeconds = new double[count];

    for (int i = 0; i < count; i++) {
      final String name = cameraNames[i];
      LimelightHelpers.SetIMUMode(name, 3);
      LimelightHelpers.setPipelineIndex(name, 0);
      LimelightHelpers.SetIMUAssistAlpha(name, 0.005);
      LimelightHelpers.SetRobotOrientation(name, lastHeadingDeg, 0, 0, 0, 0, 0);

      inputs.connected[i] = true;
      inputs.hasTarget[i] = LimelightHelpers.getTV(name);
      inputs.txDegrees[i] = LimelightHelpers.getTX(name);
      inputs.tyDegrees[i] = LimelightHelpers.getTY(name);
      inputs.taPercent[i] = LimelightHelpers.getTA(name);
      inputs.fiducialId[i] = LimelightHelpers.getFiducialID(name);
      inputs.pipelineLatencyMs[i] = LimelightHelpers.getLatency_Pipeline(name);
      inputs.captureLatencyMs[i] = LimelightHelpers.getLatency_Capture(name);

      final PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      if (estimate != null && estimate.tagCount > 0) {
        inputs.tagCount[i] = estimate.tagCount;
        inputs.avgTagDistMeters[i] = estimate.avgTagDist;
        inputs.poseX[i] = estimate.pose.getX();
        inputs.poseY[i] = estimate.pose.getY();
        inputs.poseThetaRad[i] = estimate.pose.getRotation().getRadians();
        inputs.timestampSeconds[i] = estimate.timestampSeconds;
      } else {
        inputs.tagCount[i] = 0.0;
        inputs.avgTagDistMeters[i] = Double.NaN;
        inputs.poseX[i] = Double.NaN;
        inputs.poseY[i] = Double.NaN;
        inputs.poseThetaRad[i] = Double.NaN;
        inputs.timestampSeconds[i] = Double.NaN;
      }
    }
  }

  @Override
  public void setRobotOrientationDegrees(double headingDeg) {
    lastHeadingDeg = headingDeg;
  }

  @Override
  public String[] getCameraNames() {
    return cameraNames.clone();
  }
}
