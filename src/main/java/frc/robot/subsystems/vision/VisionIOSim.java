package frc.robot.subsystems.vision;

/** Sim/replay-safe vision IO that keeps deterministic shape for camera arrays. */
public class VisionIOSim implements VisionIO {
  private final String[] cameraNames;

  /** Uses the provided camera names for deterministic replay indexing. */
  public VisionIOSim(String[] cameraNames) {
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
      inputs.connected[i] = true;
      inputs.avgTagDistMeters[i] = Double.NaN;
      inputs.poseX[i] = Double.NaN;
      inputs.poseY[i] = Double.NaN;
      inputs.poseThetaRad[i] = Double.NaN;
      inputs.timestampSeconds[i] = Double.NaN;
    }
  }

  @Override
  public String[] getCameraNames() {
    return cameraNames.clone();
  }
}
