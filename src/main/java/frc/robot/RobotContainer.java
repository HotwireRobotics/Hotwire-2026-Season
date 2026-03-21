package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indication.LuminalArray;
import frc.robot.subsystems.indication.limelights.LimelightArray;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Declare subsystems.
  public final Drive drive;
  public final Intake intake;
  public final Shooter shooter;
  public final Hopper hopper;
  public final LuminalArray lights;
  public final LimelightArray vision;

  // Static configuration.
  private final boolean firstPerson = false;

  // Alignment supplier.
  public final BooleanSupplier aligned;
  // Velocity supplier.
  public final Supplier<AngularVelocity> velocity;

  // Velocity control states.
  public enum VelocityType {
    STATIC,
    REGRESSION,
    TESTING,
    AUTO
  }

  // Mutable state control.
  public boolean inverse = false;
  public double testVelocity = 0;
  public final Supplier<Integer> kInverse = () -> (inverse ? -1 : 1);
  public VelocityType velocityType = VelocityType.STATIC;

  // Methodic toggles.
  private final Command velocity(VelocityType type) {
    return Commands.runOnce(() -> velocityType = type);
  }

  private final Command invertion(boolean value) {
    return Commands.runOnce(() -> inverse = value);
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize drive subsystem.
    drive = new Drive(Constants.currentMode);

    // Alignment supplier.
    aligned =
        () -> {
          return drive
              .getRotation()
              .getMeasure()
              .isNear(drive.getRotationTarget().getMeasure(), Constants.Shooter.kAlignmentError);
        };

    // Velocity supplier.
    velocity =
        () -> {
          switch (velocityType) {
            case STATIC:
              // Ferrying static velocity.
              return Constants.Shooter.kSpeed;
            case REGRESSION:
              // Conditional shooting.
              if (aligned.getAsBoolean() || !Dashboard.alignmentRequirement.get()) {
                return Constants.regress(
                    Meters.of(
                        drive
                            .getPose()
                            .minus(Constants.Poses.hub.get())
                            .getTranslation()
                            .getNorm()));
              } else {
                return Constants.Shooter.kZero;
              }
            case TESTING:
              // Allow testing of shooter velocity via dashboard input, for characterization
              // purposes.
              return RPM.of(SmartDashboard.getNumber("Test Shooter RPM", testVelocity));
            case AUTO:
              return RPM.of(1250);
            default:
              // Fallback velocity.
              return Constants.Shooter.kSpeed;
          }
        };

    // Initialize shooter subsystem.
    shooter = new Shooter(() -> velocity.get().times(kInverse.get()));

    // Initialize fuel intake and storage subsystems.
    intake = new Intake(() -> Constants.Intake.kSpeed * kInverse.get());
    hopper = new Hopper(() -> Constants.Hopper.kSpeed * kInverse.get());

    // Initialize indicator subsystems.
    lights = new LuminalArray();
    vision = new LimelightArray(drive::getPose, drive::getRotation, drive::addVisionMeasurement);

    // Configure button bindings.
    configureButtonBindings();

    // Wrist commands.
    final Command raiseWrist = intake.raiseWrist(Degrees.of(60));
    final Command lowerWrist = intake.lowerWrist().andThen(Commands.waitTime(Seconds.of(0.5)));
    final Command oscillateIntakek35 =
        intake.oscillateArm(Degrees.of(35), Constants.Intake.kOscillationFrequency);
    final Command oscillateIntakek60 =
        intake.oscillateArm(Degrees.of(60), Constants.Intake.kOscillationFrequency);
    final Command oscillateIntakek20 =
        intake.oscillateArm(Degrees.of(20), Constants.Intake.kOscillationFrequency);

    // Drivetrain commands.
    final Command stopDrive = Commands.runOnce(() -> drive.stop());
    final Command lockDrive = Commands.runOnce(() -> drive.stopWithX());

    // Shooter commands.
    final Command initializeFiring =
        Commands.sequence(lockDrive, velocity(VelocityType.REGRESSION), shooter.run());

    final Command initializeFeeding =
        Commands.sequence(Commands.waitTime(Constants.Shooter.kChargeUpTime), hopper.run());

    final Command oscillateIntakeSequence =
        oscillateIntakek20
            .raceWith(Commands.waitTime(Constants.Shooter.k35Time))
            .andThen(oscillateIntakek35.raceWith(Commands.waitTime(Constants.Shooter.k60Time)))
            .andThen(oscillateIntakek60);

    final Command terminateFiring =
        Commands.parallel(shooter.halt(), hopper.halt(), lowerWrist, velocity(VelocityType.STATIC));

    final Command runFiringSequence =
        Commands.sequence(
            initializeFiring,
            initializeFeeding,
            Commands.waitTime(Constants.Shooter.kFiringTime).raceWith(oscillateIntakeSequence),
            terminateFiring);

    // Register commands for pathplanner.
    NamedCommands.registerCommand("Firing Sequence", runFiringSequence);
    NamedCommands.registerCommand("Start Intaking", intake.run());
    NamedCommands.registerCommand("Stop Intaking", intake.halt());
    NamedCommands.registerCommand(
        "Intake Period", intake.run().repeatedly().finallyDo(() -> intake.halt()));
    NamedCommands.registerCommand("Raise Intake", raiseWrist);
    NamedCommands.registerCommand("Lower Intake", lowerWrist);
    NamedCommands.registerCommand("Stop", stopDrive);

    // Create autonomous selector and add options.
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Drivetrain characterization routines.
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

    // Shooter characterization routines.
    autoChooser.addOption("Right Shooter SysId", shooter.sysIdRightAnalysis());
    autoChooser.addOption("Left Shooter SysId", shooter.sysIdLeftAnalysis());
    autoChooser.addOption("Feeder SysId", shooter.sysIdFeederAnalysis());

    // Secondary autonomous routine.
    autoChooser.addOption("A-Bineutral Right", new PathPlannerAuto("A-Bineutral", false));
    autoChooser.addOption("A-Bineutral Left", new PathPlannerAuto("A-Bineutral", true));

    // Primary autonomous routine.
    autoChooser.addOption("A-Unineutral Right", new PathPlannerAuto("A-Unineutral", false));
    autoChooser.addOption("A-Unineutral Left", new PathPlannerAuto("A-Unineutral", true));

    /** Test autonomous firing sequence. */
    autoChooser.addOption("Shooting Sequence", runFiringSequence);
  }

  /** Returns the Rotation2d the robot needs to face the hub. */
  private Rotation2d calculateHubRotation() {
    // Get poses.
    Pose2d robotPose = drive.getPose();
    Pose2d hubPose = Constants.Poses.hub.get();

    // Pose differences.
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();

    // Angle from robot to hub
    Angle toHub = Constants.allianceRelative(Radians.of(Math.IEEEremainder(Math.atan2(dy, dx), Constants.Mathematics.TAU)));

    // Log the pointer
    Pose2d pointer = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(toHub));
    Logger.recordOutput("Hub Pointer", pointer);

    // Update drive target.
    drive.setRotationTarget(new Rotation2d(toHub).rotateBy(Rotation2d.k180deg));

    return drive.getRotationTarget();
  }

  /** Orient robot to face the hub. */
  private Command firingOrientation() {
    return pointToAngle(this::calculateHubRotation);
  }

  /**
   * Orient the robot to face a supplied angle.
   *
   * @param rotation
   */
  private Command pointToAngle(Supplier<Rotation2d> rotation) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -Constants.Joysticks.driver.getLeftY(),
        () -> -Constants.Joysticks.driver.getLeftX(),
        rotation);
  }

  private void configureButtonBindings() {
    if (firstPerson) {
      // First person drive command.
      drive.setDefaultCommand(
          DriveCommands.firstPersonDrive(
              drive,
              () -> -Constants.Joysticks.operator.getLeftY(),
              () -> -Constants.Joysticks.operator.getLeftX(),
              () -> -Constants.Joysticks.operator.getRightX()));
    } else {
      // Third person drive command.
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -Constants.Joysticks.driver.getLeftY(),
              () -> -Constants.Joysticks.driver.getLeftX(),
              () -> -Constants.Joysticks.driver.getRightX()));
    }

    // Hold wheel position.
    Constants.Joysticks.driver.rightBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Seed limelights and zero pose heading.
    Constants.Joysticks.driver
        .a()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                      for (String limelight : Constants.Limelight.localization) {
                        LimelightHelpers.SetIMUMode(limelight, 0);
                        LimelightHelpers.SetRobotOrientation(limelight, 0, 0, 0, 0, 0, 0);
                        LimelightHelpers.SetIMUMode(limelight, 2);
                      }
                    },
                    drive)
                .ignoringDisable(true));

    // Toggle intake between raised and lowered positions to aggitate fuel.
    Constants.Joysticks.operator
        .povUp()
        .onFalse(intake.lowerWrist().alongWith(intake.halt()))
        .onTrue(intake.raiseWrist(Degrees.of(60)).alongWith(intake.run()));

    // Run intake rollers at full speed when left trigger is held, and halt when released.
    Constants.Joysticks.operator.leftTrigger().onFalse(intake.halt()).onTrue(intake.run());

    // Run shooter at target velocity when right bumper is held, and halt when released.
    Constants.Joysticks.operator
        .rightBumper()
        .whileTrue(
          shooter.run().alongWith(Commands.either(
            hopper.run(), hopper.halt(), () -> shooter.isReady())))
        .onFalse(
          shooter.halt().alongWith(hopper.halt()));

    // Run feeding mechanism.
    Constants.Joysticks.operator.leftBumper().whileFalse(hopper.halt()).whileTrue(hopper.run());

    // Raise intake to avoid impact.
    Constants.Joysticks.operator.povRight().onFalse(intake.lowerWrist()).onTrue(intake.emergency());

    // Invert all control.
    Constants.Joysticks.operator.povLeft().whileTrue(invertion(true)).whileFalse(invertion(false));

    // Toggle regression velocity control.
    Constants.Joysticks.operator
        .x()
        .whileTrue(velocity(VelocityType.REGRESSION).alongWith(firingOrientation()))
        .whileFalse(velocity(VelocityType.STATIC));
  }

  /**
   * Supplies the autonomous command selected on the dashboard.
   *
   * @return
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Set the robot's pose to the starting pose of the selected autonomous command, if it exists.
   *
   * @param autonomousCommand
   */
  public void seedAutonomousPose(Command autonomousCommand) {
    if (!(autonomousCommand instanceof PathPlannerAuto selectedAuto)) {
      return;
    }

    // Get autonomous starting pose.
    Pose2d startingPose = selectedAuto.getStartingPose();
    if (startingPose == null) {
      return;
    }

    drive.setPose(startingPose);
    Logger.recordOutput("AutoSeedPose", startingPose);
  }
}

// ./gradlew deploy --no-daemon
