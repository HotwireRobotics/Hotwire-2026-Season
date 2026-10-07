package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightHelpers;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.indication.limelights.LimelightArray;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer container;
  public Pose2d poseEstimate = new Pose2d();

  private Time time = Seconds.of(0);

  private final Field2d field = new Field2d();

  public Robot() {
    // Record metadata.
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

    // Set up data receivers and replay source.
    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start logging.
    Logger.start();

    // Initialize robot container.
    container = new RobotContainer();

    // Robot test configuration.
    SmartDashboard.putNumber("Test Shooter RPM", container.testVelocity);
    SmartDashboard.setPersistent("Test Shooter RPM");
    SmartDashboard.putData("Robot Pose (Field)", field);
  }

  private enum Indicate {
    DISABLED,
    ENABLED,
    AUTO
  }

  public void indicateLimelight(Indicate mode) {
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
    // Track time.
    time = Constants.Tempo.tick();

    // Control command scheduler and log data.
    CommandScheduler.getInstance().run();

    // Log poses.
    Logger.recordOutput("Hub Pose", Constants.Poses.hub.getPose());
    Logger.recordOutput("Tower Pose", Constants.Poses.tower.getPose());
    Logger.recordOutput("Robot Pose", container.drive.getPose());
    Logger.recordOutput("Is Neutral", container.drive.isNeutralZone());

    // Log shooter status.
    Logger.recordOutput("Shooter/aligned", container.aligned);
    Logger.recordOutput("Shooter/ready", container.shooter.isReady());
    Logger.recordOutput("Shooter/target", container.velocity.get().in(RotationsPerSecond));
    Logs.write("Shooter/type", container.velocityType);

    // Update python pose estimate.
    Double[] robotpose = {
      container.drive.getPose().getX(), container.drive.getPose().getX(), container.drive.getRotation().getDegrees()
    };
    SmartDashboard.putNumberArray("robot-pose", robotpose);

    // Update field visualization.
    field.setRobotPose(container.drive.getPose());
    if (container.simulation != null) {
      container.simulation.tick();
    }
  }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");
    container.releaseShootOnFly();
  }

  @Override
  public void disabledPeriodic() {
    indicateLimelight(Indicate.DISABLED);
    container.vision.setIMUMode(LimelightArray.IMUMode.OFF);
  }

  @Override
  public void autonomousInit() {
    Logger.recordOutput("Robot/Mode", "Autonomous");
    autonomousCommand = container.getAutonomousCommand();
    container.seedAutonomousPose(autonomousCommand);

    if (autonomousCommand != null) {
      Logger.recordOutput("Robot/AutonomousCommand", autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      Logger.recordOutput("Robot/AutonomousCommand", "None");
    }
    Constants.Tempo.startTime();
    if (container.simulation != null) {
      container.simulation.autonomous();
    }
  }

  @Override
  public void autonomousPeriodic() {
    indicateLimelight(Indicate.AUTO);
    container.vision.setIMUMode(LimelightArray.IMUMode.OFF);
  }

  @Override
  public void teleopInit() {
    Logger.recordOutput("Robot/Mode", "Teleop");
    container.inverse = false;
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
    container.vision.setIMUMode(LimelightArray.IMUMode.OFF);
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
