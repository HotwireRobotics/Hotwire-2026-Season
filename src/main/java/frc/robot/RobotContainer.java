package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.ProtoIntake;
import frc.robot.subsystems.shooter.ProtoShooter;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final ProtoIntake intake;
  public final ProtoShooter shooter;
  public double feederVelocity = 0;
  public double shooterVelocity = 0;
  public double shooterPower = 0;

  // Constants.Joysticks

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

    SmartDashboard.putNumber("Feeder Velocity", feederVelocity);
    SmartDashboard.putNumber("Shooter Velocity", shooterVelocity);

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -Constants.Joysticks.driver.getLeftY(),
            () -> -Constants.Joysticks.driver.getLeftX(),
            () -> -Constants.Joysticks.driver.getRightX()));
    Constants.Joysticks.operator
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -Constants.Joysticks.driver.getLeftY(),
                () -> -Constants.Joysticks.driver.getLeftX(),
                () -> {
                  Pose2d robotPose = drive.getPose();
                  Pose2d hubPose = Constants.Poses.hub;

                  // Calculate the angle from the robot to the hub
                  double hubDirection =
                      Math.atan(
                          (hubPose.getY() - robotPose.getY())
                              / (hubPose.getX() - robotPose.getX()));

                  Angle toHub = Radians.of(Math.IEEEremainder(hubDirection, 2 * Math.PI));
                  Logger.recordOutput("Angle to Hub", toHub.in(Degrees));

                  return new Rotation2d(toHub);
                }));

    // Lock to 0° when down POV button is helds
    Constants.Joysticks.driver
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Constants.Joysticks.driver.getLeftY(),
                () -> Constants.Joysticks.driver.getLeftX(),
                () -> Rotation2d.kZero));

    // Hold wheel position.
    Constants.Joysticks.driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when A button is pressed.
    // Constants.Joysticks.driver
    //     .a()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive).ignoringDisable(true));

    Constants.Joysticks.driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                      for (String limelight : Constants.limelights) {
                        LimelightHelpers.SetIMUMode(limelight, 1);
                        LimelightHelpers.SetRobotOrientation(limelight, 0, 0, 0, 0, 0, 0);
                        LimelightHelpers.SetIMUMode(limelight, 2);
                      }
                    },
                    drive)
                .ignoringDisable(true));

    // Constants.Joysticks.operator
    //     .a()
    //     .whileTrue(intake.runRollersPercent(0.4))
    //     .onFalse(intake.runRollersPercent(0.0));
    Constants.Joysticks.operator
        .rightBumper()
        .whileTrue(
            // Use range (1 < n ≤ 100) or (0 ≤ n ≤ 1)
            shooter.runShooterAndFeeder(
                (Math.abs(shooterPower) > 1) ? shooterPower / 100 : shooterPower))
        .onFalse(shooter.runShooterAndFeeder(0));

    List<Pose2d> towerPoses = new ArrayList<Pose2d>();
    towerPoses.add(
        Constants.Poses.tower.transformBy(
            new Transform2d(Meters.of(0.5), Meters.of(0), Rotation2d.kZero)));
    towerPoses.add(Constants.Poses.tower);
    Constants.Joysticks.driver
        .povUp()
        .whileTrue(DriveCommands.pathfind(drive, towerPoses, Constants.constraints));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
