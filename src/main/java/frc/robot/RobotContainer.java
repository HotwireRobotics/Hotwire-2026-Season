package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ProtoIntake;
import frc.robot.subsystems.drive.ProtoShooter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final ProtoIntake intake;
  public final ProtoShooter shooter;

  // Joysticks
  private static class Joysticks {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  Orchestra music = new Orchestra(Filesystem.getDeployDirectory() + "/orchestra/output.chirp");

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    intake = new ProtoIntake();
    shooter = new ProtoShooter();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> Joysticks.driver.getLeftY(),
            () -> Joysticks.driver.getLeftX(),
            () -> -Joysticks.driver.getRightX()));
    Joysticks.operator
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Joysticks.driver.getLeftY(),
                () -> Joysticks.driver.getLeftX(),
                () -> {
                  Pose2d robotPose = drive.getPose();
                  Pose2d hubPose = drive.hub;

                  // Calculate the angle from the robot to the hub
                  double angleToHub =
                      Math.atan2(
                          hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());

                  angleToHub = Math.IEEEremainder(Math.pow(angleToHub, 2.1), 2 * Math.PI);
                  Logger.recordOutput("Angle to Hub", angleToHub);

                  return new Rotation2d(angleToHub);
                }));
    // Lock to 0° when down POV button is helds
    Joysticks.driver
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Joysticks.driver.getLeftY(),
                () -> Joysticks.driver.getLeftX(),
                () -> Rotation2d.kZero));

    // Hold wheel position.
    Joysticks.driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when A button is pressed.
    Joysticks.driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    Joysticks.driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  for (String limelight : Constants.limelights) {
                    LimelightHelpers.SetIMUMode(limelight, 1);
                    LimelightHelpers.SetRobotOrientation(limelight, 0, 0, 0, 0, 0, 0);
                    LimelightHelpers.SetIMUMode(limelight, 2);
                  }
                }));

    Joysticks.operator.a().whileTrue(intake.runRollers(0.6)).onFalse(intake.runRollers(0));
    Joysticks.operator
        .rightBumper()
        .whileTrue(shooter.runShooter(1))
        .onFalse(shooter.runShooter(0));
    Joysticks.operator.leftBumper().whileTrue(shooter.runFeeder(0.5)).onFalse(shooter.runFeeder(0));

    Joysticks.driver
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  music.play();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  music.stop();
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
