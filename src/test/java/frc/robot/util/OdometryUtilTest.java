package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Unit tests for odometry sample bounding behavior. */
class OdometryUtilTest {
  @Test
  void boundedSampleCountUsesSmallestAvailableSource() {
    int bounded = OdometryUtil.boundedSampleCount(10, new int[] {8, 7, 9, 6}, 5);
    assertEquals(5, bounded);
  }

  @Test
  void boundedSampleCountNeverGoesNegative() {
    int bounded = OdometryUtil.boundedSampleCount(-1, new int[] {4, 4, 4, 4}, 4);
    assertEquals(0, bounded);
  }
}
