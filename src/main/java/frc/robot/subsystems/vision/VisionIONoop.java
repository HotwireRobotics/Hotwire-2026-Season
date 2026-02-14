package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** No-op vision IO for sim/replay when the camera is not present. */
public class VisionIONoop implements VisionIO {
  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d robotPoseForCamera) {
    inputs.valid = false;
    inputs.pose = new Pose2d();
    inputs.timestampSeconds = 0.0;
    inputs.tagCount = 0;
    inputs.avgTagDist = 0.0;
  }
}
