package frc.robot.subsystems.indication.limelights;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightHelpers;
import frc.robot.constants.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LimelightArray extends SubsystemBase {

  private static class Configuration {
    public static final Distance maxDistance = Inches.of(100);
    private static final Pipeline pipeline = Pipeline.MEGATAG2;
  }

  /** Define assist mode for the internal IMU. */
  public static enum IMUMode {
    OFF(0),
    SEED(1),
    INTERNAL(2),
    MT1ASSIST(3),
    EXTERNAL(4);

    public final int mode;

    IMUMode(int mode) {
      this.mode = mode;
    }
  }

  private int mode = 0;

  /** Measurement pipeline choice. */
  public enum Pipeline {
    MEGATAG2,
    MEGATAG1
  }

  private Pose2d lastPoseEstimate;

  // Suppliers for rotation and pose.
  private final Supplier<Pose2d> pose;
  private final Supplier<Rotation2d> gyro;

  // Consumer for pose estimates and their standard deviations.
  private final BiConsumer<PoseEstimate, Matrix<N3, N1>> supply;

  public LimelightArray(
      Supplier<Pose2d> getPose,
      Supplier<Rotation2d> getGyro,
      BiConsumer<PoseEstimate, Matrix<N3, N1>> supplyMeasurement) {
    // Suppliers
    this.pose = getPose;
    this.gyro = getGyro;
    // Consumer
    this.supply = supplyMeasurement;
    // Initialize limelights
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.setPipelineIndex(limelight, 0);
      LimelightHelpers.SetIMUAssistAlpha(limelight, 0.005);
    }
  }

  /**
   * Set the alpha value for IMU assist on all limelights. Higher values will cause the internal IMU
   * to converge onto the assist source more rapidly.
   *
   * @param alpha
   */
  public void setIMUAssistAlpha(double alpha) {
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.SetIMUAssistAlpha(limelight, alpha);
    }
  }

  /**
   * Set the IMU assist source for all limelights.
   *
   * @param mode
   */
  public void setIMUMode(IMUMode mode) {
    this.mode = mode.mode;
  }

  @Override
  public void periodic() {
    processMeasurements();
  }

  /**
   * Process measurements from all limelights, updating the pose estimate and logging data. Valid
   * measurements are supplied to the consumer along with a standard deviation matrix.
   */
  private void processMeasurements() {

    List<PoseEstimate> measurements = new ArrayList<>();
    double heading = this.gyro.get().getMeasure().in(Degrees);

    for (String limelight : Constants.Limelight.localization) {

      // Configure periodally.
      LimelightHelpers.SetIMUMode(limelight, mode);

      LimelightHelpers.SetRobotOrientation(limelight, heading, 0, 0, 0, 0, 0);

      PoseEstimate MT2estimate = getEstimation(limelight);

      // Validate estimate.
      if (isValidEstimate(MT2estimate)) {
        measurements.add(MT2estimate);
        lastPoseEstimate = MT2estimate.pose;

        // Log detecting status and pose estimate.
        Logger.recordOutput(limelight + " Detecting", true);
        Logger.recordOutput("Limelight/" + limelight + "/Pose", MT2estimate.pose);

        // Supply measurement to consumer with defined standard deviations.
        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);
        if (Dashboard.visionEnabled.get()) this.supply.accept(MT2estimate, stdDevs);
      } else {
        Logger.recordOutput(limelight + " Detecting", false);
      }
    }
  }

  /**
   * Determine whether a given pose estimate is valid based on the number of detected tags and the
   * average tag distance.
   *
   * @param estimation
   * @return
   */
  public boolean isValidEstimate(PoseEstimate estimation) {
    if (estimation == null) return false;
    return ((estimation.tagCount > 0)
        && (estimation.avgTagDist <= Configuration.maxDistance.in(Meters)));
  }

  /**
   * Get the last valid pose estimate from the limelight. Returns null if no valid estimates have
   * been recorded.
   *
   * @return
   */
  public Pose2d getLastPoseEstimate() {
    return lastPoseEstimate;
  }

  /**
   * Get the pose estimate from the limelight based on the current pipeline. Returns null if no
   * valid targets are detected.
   *
   * @param limelight
   * @return
   */
  private PoseEstimate getEstimation(String limelight) {
    return Configuration.pipeline.equals(Pipeline.MEGATAG2)
        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
  }
}
