package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indication.IndicatorsIO;
import frc.robot.subsystems.indication.IndicatorsIOCANdle;
import frc.robot.subsystems.indication.IndicatorsIOSim;
import frc.robot.subsystems.indication.LuminalIndicators;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ArmState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Intake intake;
  public final Shooter shooter;
  public final HopperSubsystem hopper;
  public final LuminalIndicators lights;
  public double testVelocity = 0;
  private final Supplier<AngularVelocity> velocity; // deployprogramStartfrcJavaroborio
  public final BooleanSupplier aligned;

  public Pose2d hubTarget;

  private final boolean firstPerson = false;

  public enum VelocityType {
    STATIC,
    REGRESSION,
    TESTING
  }

  public VelocityType velocityType = VelocityType.REGRESSION;

  private void staticVelocity() {
    velocityType = VelocityType.STATIC;
  }

  private void regressVelocity() {
    velocityType = VelocityType.REGRESSION;
  }

  private void testVelocity() {
    velocityType = VelocityType.TESTING;
  }

  /** Only read driver stick when connected to avoid repeated "joystick not available" warnings. */
  private boolean driverConnected() {
    return DriverStation.isJoystickConnected(0);
  }

  private boolean operatorConnected() {
    return DriverStation.isJoystickConnected(1);
  }

  private double driverLeftY() {
    return driverConnected() ? -Constants.Joysticks.driver.getLeftY() : 0.0;
  }

  private double driverLeftX() {
    return driverConnected() ? -Constants.Joysticks.driver.getLeftX() : 0.0;
  }

  private double driverRightX() {
    return driverConnected() ? -Constants.Joysticks.driver.getRightX() : 0.0;
  }

  private double operatorLeftY() {
    return operatorConnected() ? -Constants.Joysticks.operator.getLeftY() : 0.0;
  }

  private double operatorLeftX() {
    return operatorConnected() ? -Constants.Joysticks.operator.getLeftX() : 0.0;
  }

  private double operatorRightX() {
    return operatorConnected() ? -Constants.Joysticks.operator.getRightX() : 0.0;
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  Orchestra music = new Orchestra(Filesystem.getDeployDirectory() + "/orchestra/output.chirp");

  public RobotContainer() {
    final IntakeIO intakeIO;
    final ShooterIO shooterIO;
    final HopperIO hopperIO;
    final IndicatorsIO indicatorsIO;

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intakeIO = new IntakeIOTalonFX();
        shooterIO = new ShooterIOTalonFX();
        hopperIO = new HopperIOTalonFX();
        indicatorsIO = new IndicatorsIOCANdle();
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intakeIO = new IntakeIOSim();
        shooterIO = new ShooterIOSim();
        hopperIO = new HopperIOSim();
        indicatorsIO = new IndicatorsIOSim();
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intakeIO = new IntakeIO() {};
        shooterIO = new ShooterIO() {};
        hopperIO = new HopperIO() {};
        indicatorsIO = new IndicatorsIO() {};
        break;
    }

    aligned =
        () -> {
          Rotation2d difference = drive.getRotation().minus(drive.getRotationTarget());
          return Degrees.of(Math.abs(difference.getMeasure().in(Degrees)))
              .lt(Constants.Shooter.kAlignmentError);
        };

    intake = new Intake(intakeIO);
    shooter = new Shooter(shooterIO);
    hopper = new HopperSubsystem(hopperIO);
    lights = new LuminalIndicators(indicatorsIO);

    velocity =
        () -> {
          switch (velocityType) {
            case STATIC:
              return Constants.Shooter.kSpeed;
            case REGRESSION:
              return Constants.regress(
                  Meters.of(drive.getPose().minus(hubTarget).getTranslation().getNorm()));
            case TESTING:
              return RPM.of(SmartDashboard.getNumber("Test Shooter RPM", testVelocity));
            default:
              return RPM.of(0);
          }
        };

    configureButtonBindings();

    final Command startHopper = hopper.runHopper(Constants.Hopper.kSpeed);
    final Command startShooter = conditionalShooting();
    final Command startIntake = intake.runIntake(Constants.Intake.kSpeed);
    final Command dropArm =
        intake
            .controlArm(ArmState.BACKWARD)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.controlArm(ArmState.ZERO));
    //// NamedCommands.registerCommand("StartShooter", regressionShooting().repeatedly());
    final Command killHopper = hopper.runHopper(0);
    final Command killShooter = shooter.runMechanism(0, 0);
    final Command killIntake = intake.runIntake(0);
    //// NamedCommands.registerCommand("KillShooter", killShooter);
    final Command periodIntake =
        intake.runIntake(Constants.Intake.kSpeed).repeatedly().finallyDo(() -> intake.runIntake(0));
    final Command raiseIntake = intake.raiseArm();
    final Command lowerIntake = intake.lowerArm().andThen(Commands.waitTime(Seconds.of(0.5)));
    final Command occilateIntake = intake.occilateArm(Constants.Intake.kOccilationFrequency);
    final Command stopDrive = Commands.runOnce(() -> drive.stop());
    final Command runFiringSequence =
        new SequentialCommandGroup(
            Commands.runOnce(() -> regressVelocity()),
            startShooter,
            Commands.waitTime(Constants.Shooter.kChargeUpTime),
            startHopper,
            Commands.waitTime(Constants.Shooter.kFiringTime)
                .raceWith(
                    Commands.waitTime(Constants.Shooter.kUntilAggitateTime)
                        .andThen(occilateIntake)),
            killShooter,
            killHopper,
            lowerIntake,
            Commands.runOnce(() -> staticVelocity()));
    NamedCommands.registerCommand("Firing Sequence", runFiringSequence);
    NamedCommands.registerCommand("Start Intaking", startIntake);
    NamedCommands.registerCommand("Stop Intaking", killIntake);
    NamedCommands.registerCommand("Intake Period", periodIntake);
    NamedCommands.registerCommand("Raise Intake", raiseIntake);
    NamedCommands.registerCommand("Lower Intake", lowerIntake);
    NamedCommands.registerCommand("Drop Arm", dropArm);
    NamedCommands.registerCommand("Stop", stopDrive);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooserSafe());

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

    hubTarget = Constants.Poses.hub;
  }

  private Command pointToHub() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        this::driverLeftY,
        this::driverLeftX,
        () -> {
          Pose2d robotPose = drive.getPose();
          Pose2d hubPose = Constants.Poses.hub;

          Angle toHub =
              Radians.of(
                  Math.IEEEremainder(
                      Math.atan(
                          (hubPose.getY() - robotPose.getY())
                              / (hubPose.getX() - robotPose.getX())),
                      Constants.Mathematics.TAU));
          Pose2d pointer =
              new Pose2d(robotPose.getMeasureX(), robotPose.getMeasureY(), new Rotation2d(toHub));
          Logger.recordOutput("Hub Pointer", pointer);
          drive.setRotationTarget(new Rotation2d(toHub).rotateBy(Rotation2d.k180deg));
          return drive.getRotationTarget();
        });
  }

  private Command pointToAngle(Supplier<Angle> angle) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        this::driverLeftY,
        this::driverLeftX,
        () -> {
          return new Rotation2d(angle.get());
        });
  }

  private Command conditionalShooting() {
    return shooter.runMechanismVelocity(velocity, velocity);
  }

  private Command staticShooting(AngularVelocity v) {
    return shooter.runMechanismVelocity(v, v);
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    if (firstPerson) {
      drive.setDefaultCommand(
          DriveCommands.firstPersonDrive(
              drive, () -> operatorLeftY(), () -> operatorLeftX(), () -> operatorRightX()));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive, () -> driverLeftY(), () -> driverLeftX(), () -> driverRightX()));
    }

    // Lock to 0° when down POV button is held; only poll when driver stick connected
    new Trigger(() -> driverConnected() && Constants.Joysticks.driver.povDown().getAsBoolean())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> driverLeftY(), () -> driverLeftX(), () -> Rotation2d.kZero));

    // Hold wheel position.
    // Constants.Joysticks.driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    new Trigger(() -> driverConnected() && Constants.Joysticks.driver.a().getAsBoolean())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                      for (String limelight : Constants.Limelight.localization) {
                        LimelightHelpers.SetIMUMode(limelight, 1);
                        LimelightHelpers.SetRobotOrientation(limelight, 0, 0, 0, 0, 0, 0);
                        LimelightHelpers.SetIMUMode(limelight, 2);
                      }
                    },
                    drive)
                .ignoringDisable(true));

    new Trigger(() -> operatorConnected() && Constants.Joysticks.operator.povUp().getAsBoolean())
        .onFalse(intake.lowerArm().alongWith(intake.runIntake(0.0)))
        .onTrue(intake.raiseArm().alongWith(intake.runIntake(Constants.Intake.kSpeed)));

    new Trigger(
            () -> operatorConnected() && Constants.Joysticks.operator.leftTrigger().getAsBoolean())
        .onFalse(intake.runIntake(0.0))
        .onTrue(intake.runIntake(Constants.Intake.kSpeed));

    new Trigger(
            () -> operatorConnected() && Constants.Joysticks.operator.rightTrigger().getAsBoolean())
        .onFalse(shooter.runMechanism(0, 0).alongWith(hopper.runHopper(0)))
        .onTrue(
            new ConditionalCommand(
                conditionalShooting().alongWith(hopper.runHopper(Constants.Hopper.kSpeed)),
                shooter.runMechanism(0, 0).alongWith(hopper.runHopper(0)),
                aligned));

    new Trigger(() -> operatorConnected() && Constants.Joysticks.operator.povRight().getAsBoolean())
        .onFalse(intake.lowerArm())
        .onTrue(intake.emergency());

    new Trigger(() -> operatorConnected() && Constants.Joysticks.operator.x().getAsBoolean())
        .whileTrue(Commands.run(() -> regressVelocity()).alongWith(pointToHub()))
        .whileFalse(Commands.run(() -> staticVelocity()));
  }

  /**
   * Builds PathPlanner auto chooser; on failure (e.g. bad path causing Rotation2d error), returns
   * an empty chooser so robot startup still completes.
   */
  private static SendableChooser<Command> buildAutoChooserSafe() {
    try {
      return AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      DriverStation.reportError("Auto chooser failed: " + e.getMessage(), e.getStackTrace());
      SendableChooser<Command> empty = new SendableChooser<>();
      empty.setDefaultOption("None", Commands.none());
      return empty;
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
