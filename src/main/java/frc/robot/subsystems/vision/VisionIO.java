package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for vision (Limelight) so pose estimates are logged and replayed. One instance per
 * camera. See AdvantageKit IO Interfaces and dashboard inputs.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    /**
     * Whether a valid pose estimate was available this cycle (tagCount &gt; 0, avgTagDist &lt; 3).
     */
    public boolean valid = false;

    public Pose2d pose = new Pose2d();
    public double timestampSeconds = 0.0;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
  }

  /**
   * Updates the set of loggable inputs. Optional robot pose is used by real LL to send orientation
   * to the camera; no-op in sim/replay.
   */
  public default void updateInputs(VisionIOInputs inputs, Pose2d robotPoseForCamera) {}
}
