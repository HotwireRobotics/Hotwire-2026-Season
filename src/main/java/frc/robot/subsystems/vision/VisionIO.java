package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /** Logged limelight measurements indexed by configured camera order. */
  @AutoLog
  public static class VisionIOInputs {
    public int cameraCount = 0;
    public boolean[] connected = new boolean[] {};
    public boolean[] hasTarget = new boolean[] {};
    public double[] txDegrees = new double[] {};
    public double[] tyDegrees = new double[] {};
    public double[] taPercent = new double[] {};
    public double[] fiducialId = new double[] {};
    public double[] pipelineLatencyMs = new double[] {};
    public double[] captureLatencyMs = new double[] {};
    public double[] tagCount = new double[] {};
    public double[] avgTagDistMeters = new double[] {};
    public double[] poseX = new double[] {};
    public double[] poseY = new double[] {};
    public double[] poseThetaRad = new double[] {};
    public double[] timestampSeconds = new double[] {};
  }

  /** Updates all limelight inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Applies robot heading to all limelights for MT2 orientation assist. */
  public default void setRobotOrientationDegrees(double headingDeg) {}

  /** Returns camera names used by this IO implementation. */
  public default String[] getCameraNames() {
    return new String[] {};
  }
}
