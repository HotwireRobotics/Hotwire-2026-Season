package frc.robot.subsystems.limelights;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.Constants;
import frc.robot.constants.Field;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class LimelightArray extends SubsystemBase {

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

  private Pose2d lastPoseEstimate;
  private final Supplier<Pose2d> pose;
  private final Supplier<Rotation2d> gyro;

  private final BiConsumer<PoseEstimate, Matrix<N3, N1>> supply;

  public LimelightArray(Supplier<Pose2d> getPose, Supplier<Rotation2d> getGyro, BiConsumer<PoseEstimate, Matrix<N3, N1>> supplyMeasurement) {
    // Suppliers
    this.pose = getPose;
    this.gyro = getGyro;
    // Consumer
    this.supply = supplyMeasurement;
    // Initialize limelights
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.SetIMUMode(limelight, 3);
      LimelightHelpers.setPipelineIndex(limelight, 0);
      LimelightHelpers.SetIMUAssistAlpha(limelight, 0.005);
    }
  }

  public void setIMUAssistAlpha(double alpha) {
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.SetIMUAssistAlpha(limelight, alpha);
    }
  }

  public void setIMUMode(IMUMode mode) {
    this.mode = mode.mode;
  }

  @Override
  public void periodic() {
    processMeasurements();
  }

  private void processMeasurements() {

    List<PoseEstimate> measurements = new ArrayList<>();
    double heading = this.gyro.get().getMeasure().in(Degrees);

    for (String limelight : Constants.Limelight.localization) {

      LimelightHelpers.SetIMUMode(limelight, mode);

      LimelightHelpers.SetRobotOrientation(limelight, heading, 0, 0, 0, 0, 0);

      PoseEstimate MT2estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

      if (isValidEstimate(MT2estimate)) {
        measurements.add(MT2estimate);
        lastPoseEstimate = MT2estimate.pose;

        Logger.recordOutput(limelight + " Detecting", true);
        Logger.recordOutput("Limelight/" + limelight + "/Pose", MT2estimate.pose);

        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);
        this.supply.accept(MT2estimate, stdDevs);
      } else {
        Logger.recordOutput(limelight + " Detecting", false);
      }
    }
  }

  public boolean isValidEstimate(PoseEstimate estimation) {
    if (estimation == null) return false;
    Pose2d pose = estimation.pose;
    return (estimation.tagCount > 0) && 
           (estimation.avgTagDist <= Constants.Limelight.maxDistance.in(Meters));
  }

  public Pose2d getLastPoseEstimate() {
    return lastPoseEstimate;
  }
}
