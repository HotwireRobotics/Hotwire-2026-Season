package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
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

    // Control command scheduler and log data.
    CommandScheduler.getInstance().run();

    // Log poses.
    Logger.recordOutput("Hub Pose", Constants.Poses.hub.get());
    Logger.recordOutput("Tower Pose", Constants.Poses.tower.get());
    Logger.recordOutput("Robot Pose", robotContainer.drive.getPose());

    // Log shooter status.
    Logger.recordOutput("Shooter/aligned", robotContainer.aligned);
    Logger.recordOutput("Shooter/ready", robotContainer.shooter.isReady());
    Logs.write("Shooter/type", robotContainer.velocityType);

    // Update python pose estimate.
    Double[] robotpose = {
      robotContainer.drive.getPose().getX(), robotContainer.drive.getPose().getX()
    };
    SmartDashboard.putNumberArray("robot-pose", robotpose);

    // Update field visualization.
    field.setRobotPose(robotContainer.drive.getPose());
  }

  @Override
  public void disabledInit() {
    Logger.recordOutput("Robot/Mode", "Disabled");
  }

  @Override
  public void disabledPeriodic() {
    indicateLimelight(Indicate.DISABLED);
    robotContainer.vision.setIMUMode(LimelightArray.IMUMode.OFF);
  }

  @Override
  public void autonomousInit() {
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

  // "uwu" -brylee

  @Override
  public void autonomousPeriodic() {
    indicateLimelight(Indicate.AUTO);
    robotContainer.vision.setIMUMode(LimelightArray.IMUMode.OFF);
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
    robotContainer.vision.setIMUMode(LimelightArray.IMUMode.OFF);
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
