package frc.robot.util;

/** Utility helpers for safe odometry sample processing. */
public final class OdometryUtil {
  private OdometryUtil() {}

  /** Computes a safe odometry sample count bounded by available timestamp/module/gyro samples. */
  public static int boundedSampleCount(
      int timestampSampleCount, int[] moduleSampleCounts, int gyroSampleCount) {
    int safeCount = Math.min(timestampSampleCount, gyroSampleCount);
    for (int count : moduleSampleCounts) {
      safeCount = Math.min(safeCount, count);
    }
    return Math.max(0, safeCount);
  }
}
