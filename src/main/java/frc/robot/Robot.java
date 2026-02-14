package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public Pose2d poseEstimate = new Pose2d();

  /** Logged dashboard inputs so tuning values are replayed; see AdvantageKit dashboard inputs. */
  private final LoggedNetworkNumber feederVelocityInput =
      new LoggedNetworkNumber("Feeder Velocity", 0.0);

  private final LoggedNetworkNumber shooterVelocityInput =
      new LoggedNetworkNumber("Shooter Velocity", 0.0);
  private final LoggedNetworkNumber shooterRpmInput = new LoggedNetworkNumber("Shooter RPM", 0.0);
  private final LoggedNetworkNumber shooterProportionalInput =
      new LoggedNetworkNumber("Shooter Proportional", 0.0);

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

    SmartDashboard.putNumber("Shooter RPM", 0.0);
    SmartDashboard.putNumber("Shooter Proportional", 0.0);
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

    // Update logged dashboard inputs (logged and replayed; SmartDashboard.getNumber is not)
    feederVelocityInput.periodic();
    shooterVelocityInput.periodic();
    shooterRpmInput.periodic();
    shooterProportionalInput.periodic();
    robotContainer.feederVelocity = feederVelocityInput.get();
    robotContainer.shooterVelocity = shooterVelocityInput.get();
    robotContainer.shooterPower = shooterRpmInput.get();
    robotContainer.shooterKP = shooterProportionalInput.get();

    Logger.recordOutput("Hub Pose", Constants.Poses.hub);
    Logger.recordOutput("Tower Pose", Constants.Poses.tower);
  }

  private void processLimelightMeasurements() {
    robotContainer.applyVisionMeasurements();
    poseEstimate = robotContainer.getLastValidVisionPose();
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
