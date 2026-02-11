package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public Pose2d poseEstimate = new Pose2d();
  private double shooterKP = 0;

  Timer timer = new Timer();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers and replay source
    switch (Constants.currentMode) {
      case REAL: // Real robot
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM: // ! Not real
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    robotContainer = new RobotContainer();

    SmartDashboard.putNumber("Shooter RPM", robotContainer.shooterPower);
    SmartDashboard.putNumber("Shooter Proportional", shooterKP);
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Robot Pose", robotContainer.drive.getPose());
    CommandScheduler.getInstance().run();

    // Localization and orienttion feeding
    processLimelightMeasurements();

    // Tracking
    Time time = Seconds.of(DriverStation.getMatchTime());
    Boolean isAutonomous = DriverStation.isAutonomous();
    Time length = (isAutonomous) ? Constants.autoLength : Constants.teleopLength;
    time = (time.isEquivalent(Seconds.of(-1))) ? Seconds.of(timer.get()) : length.minus(time);

    // Controller haptic indicators
    Logger.recordOutput("Time", time.in(Seconds));
    Boolean rumble = false;

    for (Time target : ((isAutonomous) ? Constants.autoTimes : Constants.teleopTimes)) {
      double difference = target.minus(time).in(Seconds);
      if ((Math.abs(difference) < 1) && (difference < 0)) {
        rumble = true;
      }
    }

    Constants.Joysticks.driver.setRumble(RumbleType.kLeftRumble, rumble ? 1 : 0);

    robotContainer.feederVelocity = SmartDashboard.getNumber("Feeder Velocity", 0.0);
    robotContainer.shooterVelocity = SmartDashboard.getNumber("Shooter Velocity", 0.0);
    robotContainer.shooterPower = SmartDashboard.getNumber("Shooter RPM", 0.0);

    robotContainer.shooter.configureProportional(shooterKP);

    Logger.recordOutput("Hub Pose", Constants.Poses.hub);
    Logger.recordOutput("Tower Pose", Constants.Poses.tower);
  }

  private void processLimelightMeasurements() {
    List<PoseEstimate> measurements = new ArrayList<>();

    for (String limelight : Constants.limelights) {
      LimelightHelpers.SetIMUMode(limelight, 2);
      // Get current pose
      Pose2d robotPose = robotContainer.drive.getPose();
      double headingDeg = robotPose.getRotation().getDegrees();

      LimelightHelpers.setPipelineIndex(limelight, 0);

      // Get pose estimate from limelight
      PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

      if ((measurement != null) && (measurement.tagCount > 0) && (measurement.avgTagDist < 3)) {
        measurements.add(measurement);
        // Log pose estimate and limelight status
        Logger.recordOutput(limelight + " detecting", true);
        poseEstimate = measurement.pose;
        Logger.recordOutput("Pose Estimate", poseEstimate);

        // Define standard deviation
        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.05, 0.05, Math.toRadians(2));
        robotContainer.drive.addVisionMeasurement(
            measurement.pose, measurement.timestampSeconds, stdDevs);
      } else {
        Logger.recordOutput(limelight + " detecting", false);
      }

      LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);
    }
  }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");

    timer.restart();
    timer.stop();
  }

  @Override
  public void disabledPeriodic() {
    for (String limelight : Constants.limelights) {
      LimelightHelpers.SetThrottle(limelight, 150);
    }
  }

  @Override
  public void autonomousInit() {
    Logger.recordOutput("Robot/Mode", "Autonomous");
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      Logger.recordOutput("Robot/AutonomousCommand", autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      Logger.recordOutput("Robot/AutonomousCommand", "None");
    }

    timer.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Logger.recordOutput("Robot/Mode", "Teleop");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    for (String limelight : Constants.limelights) {
      LimelightHelpers.SetThrottle(limelight, 0);
    }
  }

  @Override
  public void testInit() {
    Logger.recordOutput("Robot/Mode", "Test");
    CommandScheduler.getInstance().cancelAll();

    teleopInit();

    timer.start();
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
