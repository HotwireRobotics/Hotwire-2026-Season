package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  public Pose2d poseEstimate = new Pose2d();

  private Time time = Seconds.of(0);

  private final Field2d field = new Field2d();

  // Orchestra music;

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

    SmartDashboard.putNumber("Oscillate", Constants.Intake.kOscillationFrequency.in(Hertz));
    SmartDashboard.putNumber("Test Shooter RPM", robotContainer.testVelocity);
    SmartDashboard.setPersistent("Test Shooter RPM");
    SmartDashboard.putData("Robot Pose (Field)", field);

    // music = new Orchestra();
    // music.addInstrument(robotContainer.intake.rollers);
    // music.addInstrument(robotContainer.shooter.m_feeder);
    // music.addInstrument(robotContainer.shooter.m_rightShooter);
    // music.addInstrument(robotContainer.shooter.m_leftShooter);

    // music.loadMusic(
    //
    // "C:\\Users\\HotwireProgrammer\\Documents\\Repositories\\2026Hotwire\\src\\main\\deploy\\orchestra\\output.chrp");
  }

  private enum Indicate {
    DISABLED,
    ENABLED,
    AUTO
  }

  public void indicateLimelight(Indicate mode) {
    Boolean b = (Math.floor(time.in(Seconds) * 10) % 2) == 1;
    switch (mode) {
      case DISABLED:
        for (String limelight : Constants.Limelight.limelights) {
          LimelightHelpers.setLEDMode_ForceOff(limelight);
        }
        break;
      case ENABLED:
        for (String limelight : Constants.Limelight.limelights) {
          LimelightHelpers.setLEDMode_ForceOn(limelight);
        }
        break;
      case AUTO:
        for (String limelight : Constants.Limelight.limelights) {
          LimelightHelpers.setLEDMode_ForceBlink(limelight);
        }
        break;
    }
  }

  @Override
  public void robotPeriodic() {
    time = Constants.Tempo.tick();
    robotContainer.mHertzOscillate = Hertz.of(SmartDashboard.getNumber("Oscillate", 0));
    Logger.recordOutput("Robot Pose", robotContainer.drive.getPose());
    Logger.recordOutput("Shooter/aligned", robotContainer.aligned);
    CommandScheduler.getInstance().run();

    // Controller haptic indicators
    Boolean rumble = false;

    for (Time target : Constants.Indication.transitions) {
      double difference = target.minus(time).in(Seconds);
      if ((Math.abs(difference) < 1) && (difference < 0)) {
        rumble = true;
      }
    }

    Constants.Joysticks.driver.setRumble(RumbleType.kLeftRumble, rumble ? 1 : 0);

    Logger.recordOutput("Hub Pose", Constants.Poses.hub);
    Logger.recordOutput("Tower Pose", Constants.Poses.tower);

    Logger.recordOutput("Shooting State", robotContainer.velocityType.toString());

    Double[] robotpose = {
      robotContainer.drive.getPose().getX(), robotContainer.drive.getPose().getX()
    };
    SmartDashboard.putNumberArray("robot-pose", robotpose);

    field.setRobotPose(robotContainer.drive.getPose());
  }

  // private void processLimelightMeasurements() {
  //   List<PoseEstimate> measurements = new ArrayList<>();

  //   for (String limelight : Constants.Limelight.localization) {
  //     LimelightHelpers.SetIMUMode(limelight, 3);
  //     LimelightHelpers.setPipelineIndex(limelight, 0);
  //     Pose2d robotPose = robotContainer.drive.getPose();
  //     double headingDeg = robotPose.getRotation().getDegrees();

  //     LimelightHelpers.SetIMUAssistAlpha(limelight, 0.003);

  //     // Get pose estimate from limelight
  //     PoseEstimate MG2measurement =
  // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

  //     if ((MG2measurement != null)
  //         && (MG2measurement.tagCount > 0)
  //         && (MG2measurement.avgTagDist <= Constants.Limelight.maxDistance.in(Meters))) {
  //       // measurement = MG1measurement.
  //       measurements.add(MG2measurement);
  //       // Log pose estimate and limelight status
  //       Logger.recordOutput(limelight + " Detecting", true);
  //       poseEstimate = MG2measurement.pose;
  //       Logger.recordOutput("Pose Estimate", poseEstimate);

  //       // Define standard deviation
  //       Matrix<N3, N1> stdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(20));

  //       Logger.recordOutput("limelight Estimate", MG2measurement.pose);
  //       robotContainer.drive.addVisionMeasurement(
  //           MG2measurement.pose, MG2measurement.timestampSeconds, stdDevs);
  //     } else {
  //       Logger.recordOutput(limelight + " Detecting", false);
  //     }
  //     LimelightHelpers.SetRobotOrientation(limelight, headingDeg, 0, 0, 0, 0, 0);
  //   }
  // }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");
  }

  @Override 
  public void disabledPeriodic() {
    indicateLimelight(Indicate.DISABLED);
  }

  @Override
  public void autonomousInit() { //
    Logger.recordOutput("Robot/Mode", "Autonomous");
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.seedAutonomousPose(autonomousCommand);

    if (autonomousCommand != null) {
      Logger.recordOutput("Robot/AutonomousCommand", autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      Logger.recordOutput("Robot/AutonomousCommand", "None");
    }
    Constants.Tempo.startTime();
  }

  @Override
  public void autonomousPeriodic() {
    indicateLimelight(Indicate.AUTO);
  }

  @Override
  public void teleopInit() {
    Logger.recordOutput("Robot/Mode", "Teleop");
    robotContainer.inverse = false;
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    Constants.Tempo.startTime(Seconds.of(20));
  }

  @Override
  public void teleopPeriodic() {
    for (String limelight : Constants.Limelight.localization) {
      LimelightHelpers.SetThrottle(limelight, 0);
    }
    indicateLimelight(Indicate.ENABLED);
  }

  @Override
  public void testInit() {
    Logger.recordOutput("Robot/Mode", "Test");
    CommandScheduler.getInstance().cancelAll();

    teleopInit();
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
