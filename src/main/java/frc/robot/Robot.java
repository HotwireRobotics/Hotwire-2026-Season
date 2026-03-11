package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
  private final String[] visionCameraNames;
  public Pose2d poseEstimate = new Pose2d();

  private final Timer bitimer = new Timer();
  private final Timer unitimer = new Timer();

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

    // In sim/replay, silence repeated "joystick not available" warnings when no controller is
    // plugged in
    if (Constants.currentMode == Constants.Mode.SIM
        || Constants.currentMode == Constants.Mode.REPLAY) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    robotContainer = new RobotContainer();
    switch (Constants.currentMode) {
      case REAL:
        visionIO = new VisionIOLimelight(Constants.Limelight.localization);
        break;
      case SIM:
      case REPLAY:
      default:
        visionIO = new VisionIOSim(Constants.Limelight.localization);
        break;
    }
    visionCameraNames = visionIO.getCameraNames();

    SmartDashboard.putNumber("Test Shooter RPM", robotContainer.testVelocity);
    SmartDashboard.setPersistent("Test Shooter RPM");
    SmartDashboard.putData("Robot Pose (Field)", field);

    unitimer.start();

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
    Boolean b = (Math.floor(unitimer.get() * 10) % 2) == 1;
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
    // Log driver and operator controller state to debug axis/button mappings.
    Constants.Joysticks.driver.log("Driver");
    Constants.Joysticks.operator.log("Operator");

    Logger.recordOutput("Robot Pose", robotContainer.drive.getPose());
    Logger.recordOutput("Shooter/aligned", robotContainer.aligned);
    CommandScheduler.getInstance().run();

    // Localization and orienttion feeding
    processLimelightMeasurements();

    // Tracking
    Time time = Seconds.of(DriverStation.getMatchTime());
    Boolean isAutonomous = DriverStation.isAutonomous();
    Time length = (isAutonomous) ? Constants.autoLength : Constants.teleopLength;
    time = (time.isEquivalent(Seconds.of(-1))) ? Seconds.of(bitimer.get()) : length.minus(time);

    // Controller haptic indicators
    Logger.recordOutput("Time", time.in(Seconds));
    Boolean rumble = false;

    for (Time target :
        ((isAutonomous)
            ? Constants.Indication.Autonomous.haptic
            : Constants.Indication.Teloperated.haptic)) {
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

  public boolean autonomousVictory() {
    String gameData = DriverStation.getGameSpecificMessage();
    Boolean allianceIsRed = DriverStation.getAlliance().get().equals(Alliance.Red);
    switch (gameData.charAt(0)) {
      case 'R':
        return (allianceIsRed);
      case 'B':
        return (!allianceIsRed);
      default:
        return true;
    }
  }

  private void processLimelightMeasurements() {
    Pose2d robotPose = robotContainer.drive.getPose();
    visionIO.setRobotOrientationDegrees(robotPose.getRotation().getDegrees());
    visionIO.updateInputs(visionInputs);
    Logger.processInputs("Vision", visionInputs);

    final int cameraCount = Math.min(visionInputs.cameraCount, visionCameraNames.length);
    for (int i = 0; i < cameraCount; i++) {
      final String limelight = visionCameraNames[i];
      final boolean hasEstimate = visionInputs.tagCount[i] > 0;
      final boolean inRange =
          hasEstimate
              && visionInputs.avgTagDistMeters[i] <= Constants.Limelight.maxDistance.in(Meters);

      Logger.recordOutput("Vision/" + limelight + "/Detecting", visionInputs.hasTarget[i]);
      Logger.recordOutput("Vision/" + limelight + "/TX", visionInputs.txDegrees[i]);
      Logger.recordOutput("Vision/" + limelight + "/TY", visionInputs.tyDegrees[i]);
      Logger.recordOutput("Vision/" + limelight + "/TA", visionInputs.taPercent[i]);
      Logger.recordOutput("Vision/" + limelight + "/TagCount", visionInputs.tagCount[i]);
      Logger.recordOutput(
          "Vision/" + limelight + "/AvgTagDistMeters", visionInputs.avgTagDistMeters[i]);
      Logger.recordOutput(
          "Vision/" + limelight + "/PipelineLatencyMs", visionInputs.pipelineLatencyMs[i]);
      Logger.recordOutput(
          "Vision/" + limelight + "/CaptureLatencyMs", visionInputs.captureLatencyMs[i]);

      if (inRange) {
        final Pose2d measurementPose =
            new Pose2d(
                visionInputs.poseX[i],
                visionInputs.poseY[i],
                new edu.wpi.first.math.geometry.Rotation2d(visionInputs.poseThetaRad[i]));
        poseEstimate = measurementPose;
        Logger.recordOutput("Pose Estimate", poseEstimate);
        Logger.recordOutput("Vision/" + limelight + "/EstimatedPose", measurementPose);

        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.001, 0.001, Math.toRadians(0.01));
        robotContainer.drive.addVisionMeasurement(
            measurementPose, visionInputs.timestampSeconds[i], stdDevs);
      }
    }
  }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");

    bitimer.restart();
    bitimer.stop();
  }

  @Override
  public void disabledPeriodic() {
    processLimelightMeasurements();
    indicateLimelight(Indicate.DISABLED);
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

    bitimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    processLimelightMeasurements();
    indicateLimelight(Indicate.AUTO);
  }

  @Override
  public void teleopInit() {
    Logger.recordOutput("Robot/Mode", "Teleop");
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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

    bitimer.start();
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
