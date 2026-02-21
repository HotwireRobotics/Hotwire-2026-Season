package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ProtoIntake;
import frc.robot.subsystems.shooter.ProtoShooter;
import frc.robot.subsystems.shooter.ShootingVelocityCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final ProtoIntake intake;
  public final ProtoShooter shooter;
  public final HopperSubsystem hopper;
  private double feederVelocity = 0;
  private double shooterVelocity = 0;
  private double shooterPower = 0;
  private final Supplier<AngularVelocity> velocity;
  private final ShootingVelocityCalculator shootingVelocityCalculator;

  private Pose2d hubTarget = Constants.Poses.hub;

  private enum VelocityType {
    STATIC,
    REGRESSION
  }

  private VelocityType velocityType = VelocityType.REGRESSION;

  private void staticVelocity() {
    velocityType = VelocityType.STATIC;
  }

  private void regressVelocity() {
    velocityType = VelocityType.REGRESSION;
  }

  Orchestra music = new Orchestra(Filesystem.getDeployDirectory() + "/orchestra/output.chirp");

  // Constants.Joysticks

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
    hopper = new HopperSubsystem();
    shootingVelocityCalculator = new ShootingVelocityCalculator(drive::getPose, this::getHubTarget);

    velocity =
        () -> {
          switch (velocityType) {
            case STATIC:
              return Constants.Shooter.kStaticVel;
            case REGRESSION:
              return shootingVelocityCalculator.regressionVelocity();
            default:
              return RPM.of(0);
          }
        };

    configureButtonBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    registerNamedCommands();
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
    registerAutoOptions();
  }

  /** Registers reusable named commands for autonomous routines. */
  private void registerNamedCommands() {
    final Command startHopper =
        Commands.runOnce(() -> hopper.setHopperSpeed(Constants.Hopper.kSpeed));
    final Command startShooter =
        Commands.runOnce(
            () -> {
              AngularVelocity targetVelocity = velocity.get();
              shooter.runDeviceVelocity(ProtoShooter.Device.BOTH_SHOOTER, targetVelocity);
              shooter.runDeviceVelocity(ProtoShooter.Device.FEEDER, targetVelocity);
            });
    final Command killHopper = Commands.runOnce(() -> hopper.setHopperSpeed(0.0));
    final Command killShooter =
        Commands.runOnce(
            () -> {
              shooter.runShooter(0.0);
              shooter.runFeeder(0.0);
            });

    final Command runFiringSequence =
        new SequentialCommandGroup(
            startShooter,
            Commands.waitTime(Constants.Shooter.kChargeUpTime),
            startHopper,
            Commands.waitTime(Constants.Shooter.kFiringTime),
            killShooter,
            killHopper);
    NamedCommands.registerCommand("Firing Sequence", runFiringSequence);
  }

  /** Registers dashboard auto and characterization options. */
  private void registerAutoOptions() {
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
        "Right Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistaticRight(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Right Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistaticRight(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Left Shooter SysId (Quasistatic Forward)",
        shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Left Shooter SysId (Quasistatic Reverse)",
        shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Right Shooter SysId (Dynamic Forward)",
        shooter.sysIdDynamicRight(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Right Shooter SysId (Dynamic Reverse)",
        shooter.sysIdDynamicRight(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Left Shooter SysId (Dynamic Forward)",
        shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Left Shooter SysId (Dynamic Reverse)",
        shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "SysId Right Shooter Analysis",
        new SequentialCommandGroup(
            shooter.sysIdQuasistaticRight(SysIdRoutine.Direction.kForward),
            shooter.sysIdQuasistaticRight(SysIdRoutine.Direction.kReverse),
            shooter.sysIdDynamicRight(SysIdRoutine.Direction.kForward),
            shooter.sysIdDynamicRight(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "SysId Left Shooter Analysis",
        new SequentialCommandGroup(
            shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kForward),
            shooter.sysIdQuasistaticLeft(SysIdRoutine.Direction.kReverse),
            shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kForward),
            shooter.sysIdDynamicLeft(SysIdRoutine.Direction.kReverse)));
  }

  private Command pointToHub() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -Constants.Joysticks.driver.getLeftY(),
        () -> -Constants.Joysticks.driver.getLeftX(),
        () -> {
          Pose2d robotPose = drive.getPose();
          Pose2d target = getHubTarget();
          if (robotPose != null && target != null) {
            Angle toHub =
                Radians.of(
                    Math.IEEEremainder(
                        Math.atan(
                            (target.getY() - robotPose.getY())
                                / (target.getX() - robotPose.getX())),
                        Constants.Mathematics.TAU));
            return new Rotation2d(toHub).rotateBy(Rotation2d.k180deg);
          }
          return Rotation2d.kZero;
        });
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -Constants.Joysticks.driver.getLeftY(),
            () -> -Constants.Joysticks.driver.getLeftX(),
            () -> -Constants.Joysticks.driver.getRightX()));

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

    Constants.Joysticks.operator.a().whileTrue(intake.holdIntake(Constants.Intake.kInSpeed));

    Constants.Joysticks.operator
        .rightTrigger()
        .whileTrue(shooter.holdMechanismVelocity(velocity, velocity));

    Constants.Joysticks.operator.x().onTrue(Commands.runOnce(this::regressVelocity));
    Constants.Joysticks.operator.x().whileTrue(pointToHub());
    Constants.Joysticks.operator.x().onFalse(Commands.runOnce(this::staticVelocity));

    Constants.Joysticks.operator
        .leftTrigger()
        .whileTrue(hopper.holdHopper(Constants.Hopper.kSlowSpeed));

    Constants.Joysticks.driver
        .povLeft()
        .whileTrue(
            DriveCommands.pathfind(drive, Constants.Poses.lowerStart, Constants.constraints));
  }

  public Command getAutonomousCommand() {
    Command auto = autoChooser.get();
    return auto != null ? auto : Commands.none();
  }

  public double getFeederVelocity() {
    return feederVelocity;
  }

  public void setFeederVelocity(double feederVelocity) {
    this.feederVelocity = feederVelocity;
  }

  public double getShooterVelocity() {
    return shooterVelocity;
  }

  public void setShooterVelocity(double shooterVelocity) {
    this.shooterVelocity = shooterVelocity;
  }

  public double getShooterPower() {
    return shooterPower;
  }

  public void setShooterPower(double shooterPower) {
    this.shooterPower = shooterPower;
  }

  public Pose2d getHubTarget() {
    return hubTarget;
  }

  public void setHubTarget(Pose2d hubTarget) {
    this.hubTarget = hubTarget;
  }
}
