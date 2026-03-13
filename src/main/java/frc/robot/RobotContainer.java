package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indication.LuminalArray;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.ArmState;
import frc.robot.subsystems.limelights.LimelightArray;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Intake intake;
  public final Shooter shooter;
  public final Hopper hopper;
  public final LuminalArray lights;
  public final LimelightArray vision;
  public double testVelocity = 0;
  private final Supplier<AngularVelocity> velocity; // deployprogramStartfrcJavaroborio
  public final BooleanSupplier aligned;
  public Frequency mHertzOscillate = Hertz.of(0);

  public Pose2d hubTarget;

  private final boolean firstPerson = false;

  public enum VelocityType {
    STATIC,
    REGRESSION,
    TESTING
  }

  public boolean inverse = false;
  public VelocityType velocityType = VelocityType.REGRESSION;

  private final Supplier<Boolean> isInverse = () -> inverse;

  private Command staticVelocity() {
    return Commands.runOnce(
        () -> {
          velocityType = VelocityType.STATIC;
        });
  }

  private Command regressVelocity() {
    return Commands.runOnce(
        () -> {
          velocityType = VelocityType.REGRESSION;
        });
  }

  private Command invertControl() {
    return Commands.runOnce(
        () -> {
          inverse = true;
        });
  }

  private Command regularControl() {
    return Commands.runOnce(
        () -> {
          inverse = false;
        });
  }

  private void testVelocity() {
    velocityType = VelocityType.TESTING;
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  Orchestra music = new Orchestra(Filesystem.getDeployDirectory() + "/orchestra/output.chirp");

  public RobotContainer() {
    Frequency mHertzOscillate = Constants.Intake.kOscillationFrequency;
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

    aligned =
        () -> {
          Rotation2d difference = drive.getRotation().minus(drive.getRotationTarget());
          return Degrees.of(Math.abs(difference.getMeasure().in(Degrees)))
              .lt(Constants.Shooter.kAlignmentError);
        };

    intake = new Intake();
    shooter = new Shooter();
    hopper = new Hopper();
    lights = new LuminalArray();
    vision = new LimelightArray(drive);

    velocity =
        () -> {
          switch (velocityType) {
            case STATIC:
              return Constants.Shooter.kSpeed;
            case REGRESSION:
              if (aligned.getAsBoolean()) {
                return Constants.regress(
                    Meters.of(drive.getPose().minus(hubTarget).getTranslation().getNorm()));
              } else {
                return Constants.Shooter.kSpeed;
              }
            case TESTING:
              return RPM.of(SmartDashboard.getNumber("Test Shooter RPM", testVelocity));
            default:
              return Constants.Shooter.kSpeed;
          }
        };

    configureButtonBindings();

    final Command startHopper = hopper.runHopper(isInverse);
    final Command startShooter = conditionalShooting();
    final Command startIntake = intake.runIntake(isInverse);
    final Command dropArm =
        intake
            .controlArm(ArmState.BACKWARD)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.controlArm(ArmState.ZERO));
    //// NamedCommands.registerCommand("StartShooter", regressionShooting().repeatedly());
    final Command killHopper = hopper.stopHopper();
    final Command killShooter = shooter.runMechanism(0, 0);
    final Command killIntake = intake.stopIntake();
    //// NamedCommands.registerCommand("KillShooter", killShooter);
    final Command periodIntake =
        intake.runIntake(isInverse).repeatedly().finallyDo(() -> intake.stopIntake());
    final Command raiseIntake = intake.raiseArm(Degrees.of(60));
    final Command lowerIntake = intake.lowerArm().andThen(Commands.waitTime(Seconds.of(0.5)));
    final Command oscillateIntakek35 =
        intake.oscillateArm(Degrees.of(35), Constants.Intake.kOscillationFrequency);
    final Command oscillateIntakek60 =
        intake.oscillateArm(Degrees.of(60), Constants.Intake.kOscillationFrequency);
    final Command oscillateIntakek20 =
        intake.oscillateArm(Degrees.of(20), Constants.Intake.kOscillationFrequency);
    final Command stopDrive = Commands.runOnce(() -> drive.stop());
    final Command runFiringSequence =
        new SequentialCommandGroup(
            regressVelocity(),
            startShooter,
            Commands.waitTime(Constants.Shooter.kChargeUpTime),
            startHopper,
            Commands.waitTime(Constants.Shooter.kFiringTime)
                .raceWith(
                    oscillateIntakek20
                        .raceWith(Commands.waitTime(Constants.Shooter.k35Time))
                        .andThen(
                            oscillateIntakek35
                                .raceWith(Commands.waitTime(Constants.Shooter.k60Time))
                                .andThen(oscillateIntakek60))),
            killShooter,
            killHopper,
            lowerIntake,
            staticVelocity());

    NamedCommands.registerCommand("Firing Sequence", runFiringSequence);
    NamedCommands.registerCommand("Start Intaking", startIntake);
    NamedCommands.registerCommand("Stop Intaking", killIntake);
    NamedCommands.registerCommand("Intake Period", periodIntake);
    NamedCommands.registerCommand("Raise Intake", raiseIntake);
    NamedCommands.registerCommand("Lower Intake", lowerIntake);
    NamedCommands.registerCommand("Stop", stopDrive);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    autoChooser.addOption("A-Bineutral Right", new PathPlannerAuto("A-Bineutral", false));
    autoChooser.addOption("A-Bineutral Left", new PathPlannerAuto("A-Bineutral", true));

    autoChooser.addOption("A-Unineutral Right", new PathPlannerAuto("A-Unineutral", false));
    autoChooser.addOption("A-Unineutral Left", new PathPlannerAuto("A-Unineutral", true));

    autoChooser.addOption("Shooting Sequence", runFiringSequence);

    // autoChooser.addOption("Shooter Clea  ning", shooter (RPM.of(-50)).repeatedly());

    hubTarget = Constants.Poses.hub;
  }

  private Command pointToHub() {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -Constants.Joysticks.driver.getLeftY(),
        () -> -Constants.Joysticks.driver.getLeftX(),
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
        () -> -Constants.Joysticks.driver.getLeftY(),
        () -> -Constants.Joysticks.driver.getLeftX(),
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
              drive,
              () -> -Constants.Joysticks.operator.getLeftY(),
              () -> -Constants.Joysticks.operator.getLeftX(),
              () -> -Constants.Joysticks.operator.getRightX()));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -Constants.Joysticks.driver.getLeftY(),
              () -> -Constants.Joysticks.driver.getLeftX(),
              () -> -Constants.Joysticks.driver.getRightX()));
    }

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
    // Constants.Joysticks.driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    Constants.Joysticks.driver
        .a()
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

    Constants.Joysticks.operator
        .povUp()
        .onFalse(intake.lowerArm().alongWith(intake.stopIntake()))
        .onTrue(intake.raiseArm(Degrees.of(60)).alongWith(intake.runIntake(isInverse)));

    Constants.Joysticks.operator
        .leftTrigger()
        .onFalse(intake.stopIntake())
        .onTrue(intake.runIntake(isInverse));

    Constants.Joysticks.operator
        .rightBumper()
        .whileFalse(shooter.runMechanism(0, 0))
        .onTrue(conditionalShooting());

    Constants.Joysticks.operator
        .leftBumper()
        .whileFalse(hopper.stopHopper())
        .onTrue(hopper.runHopper(isInverse));

    Constants.Joysticks.operator.povRight().onFalse(intake.lowerArm()).onTrue(intake.emergency());

    Constants.Joysticks.operator.povLeft().whileTrue(invertControl()).whileFalse(regularControl());

    Constants.Joysticks.operator
        .x()
        .whileTrue(regressVelocity().alongWith(pointToHub()))
        .whileFalse(staticVelocity());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void seedAutonomousPose(Command autonomousCommand) {
    if (!(autonomousCommand instanceof PathPlannerAuto selectedAuto)) {
      return;
    }

    Pose2d startingPose = selectedAuto.getStartingPose();
    if (startingPose == null) {
      return;
    }

    drive.setPose(startingPose);
    Logger.recordOutput("AutoSeedPose", startingPose);
  }
}

// ./gradlew deploy --no-daemon
