package frc.robot.subsystems.limelights;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LimelightArray extends SubsystemBase {

  private final Drive drive;

  private Pose2d lastPoseEstimate;

  public LimelightArray(Drive drive) {
    this.drive = drive;

    // Initialize limelights
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.SetIMUMode(limelight, 3);
      LimelightHelpers.setPipelineIndex(limelight, 0);
      LimelightHelpers.SetIMUAssistAlpha(limelight, 0.005);
    }
  }

  @Override
  public void periodic() {
    processMeasurements();
  }

  private void processMeasurements() {

    List<PoseEstimate> measurements = new ArrayList<>();

    Pose2d pose = drive.getPose();
    double heading = pose.getRotation().getDegrees();

    for (String limelight : Constants.Limelight.localization) {

      LimelightHelpers.SetRobotOrientation(limelight, heading, 0, 0, 0, 0, 0);

      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

      if (isValidMeasurement(estimate)) {

        Constants.Indication.isValidMeasurement = true;

        measurements.add(estimate);

        Logger.recordOutput(limelight + " Detecting", true);

        lastPoseEstimate = estimate.pose;

        Logger.recordOutput("Limelight/" + limelight + "/Pose", estimate.pose);

        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.25, 0.25, Math.toRadians(20));

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);

      } else {
        Logger.recordOutput(limelight + " Detecting", false);
        Constants.Indication.isValidMeasurement = false;
      }
    }
  }

  private boolean isValidMeasurement(PoseEstimate estimate) {
    return estimate != null
        && estimate.tagCount > 0
        && estimate.avgTagDist
            <= Constants.Limelight.maxDistance.in(edu.wpi.first.units.Units.Meters);
  }

  public Pose2d getLastPoseEstimate() {
    return lastPoseEstimate;
  }
}