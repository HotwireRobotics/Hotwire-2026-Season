package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightHelpers;
import frc.robot.simulation.Handler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indication.LuminalArray;
import frc.robot.subsystems.indication.Viewport;
import frc.robot.subsystems.indication.limelights.LimelightArray;
import frc.robot.subsystems.indication.limelights.LimelightArray.IMUMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HubShot;
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
  public final Viewport camera;

  /** Fuel sim. Null on the real robot and during log replay. */
  public final Handler simulation;

  // Static configuration.
  private final boolean firstPerson = false;
  private final boolean testing = false;

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

  /** True while operator X is held: heading tracks the hub and feed waits for alignment. */
  public boolean shootOnFly = false;

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
          try {
            Rotation2d measured = drive.getRotation();
            Rotation2d target = drive.getRotationTarget();
            if (measured == null
                || target == null
                || !Double.isFinite(measured.getRadians())
                || !Double.isFinite(target.getRadians())) {
              return false;
            }
            // Measure.isNear compares raw magnitudes and does not wrap.
            return HubShot.headingsAligned(
                measured, target, Constants.Shooter.kAlignmentError.in(Radians));
          } catch (RuntimeException ex) {
            return false;
          }
        };

    // Velocity supplier.
    velocity =
        () ->
            switch (velocityType) {
              case STATIC -> Constants.Shooter.kSpeed;
              case REGRESSION, AUTO -> RPM.of(currentShot().rpm);
              case TESTING -> RPM.of(SmartDashboard.getNumber("Test Shooter RPM", testVelocity));
            };

    // Initialize shooter subsystem.
    shooter = new Shooter(() -> velocity.get().times(kInverse.get()));

    // Initialize fuel intake and storage subsystems.
    intake = new Intake(() -> Constants.Intake.kSpeed * kInverse.get());
    hopper = new Hopper(() -> Constants.Hopper.kSpeed * kInverse.get());

    // Initialize indicator subsystems.
    lights = new LuminalArray();
    vision = new LimelightArray(drive::getPose, drive::getRotation, drive::addVisionMeasurement);
    camera = new Viewport(0);

    // Fuel sim reads the same muzzle as HubShot. Field velocity is what the launch adds.
    simulation =
        Constants.currentMode == Constants.Mode.SIM
            ? new Handler(
                velocity,
                () -> shooter.getState().equals(Shooter.State.FIRING),
                () -> intake.getState().equals(Intake.State.INTAKING),
                intake::getTarget,
                drive::getPose,
                drive::getFieldVelocity,
                drive::setPose)
            : null;

    // Configure button bindings.
    configureButtonBindings();

    // Wrist commands.
    final Command raiseWrist = intake.raiseWrist(Degrees.of(60));
    final Command lowerWrist = intake.lowerWrist().andThen(Commands.waitTime(Seconds.of(0.5)));
    final Command ThirdMagnitude =
        intake.oscillateArm(Rotations.of(0.17), Constants.Intake.kOscillationFrequency);
    final Command SecondMagnitude =
        intake.oscillateArm(Rotations.of(0.12), Constants.Intake.kOscillationFrequency);
    final Command FirstMagnitude =
        intake.oscillateArm(Rotations.of(0.0), Constants.Intake.kOscillationFrequency);

    // Drivetrain commands.
    final Command stopDrive = Commands.runOnce(() -> drive.stop());
    final Command lockDrive = Commands.runOnce(() -> drive.stopWithX());

    // Shooter commands.
    final Command initializeFiring =
        Commands.sequence(lockDrive, velocity(VelocityType.AUTO), shooter.run(), intake.run());

    final Command initializeFeeding =
        Commands.sequence(Commands.waitTime(Constants.Shooter.kChargeUpTime), hopper.run());

    final Command oscillateIntakeSequence =
        FirstMagnitude
            .raceWith(Commands.waitTime(Constants.Shooter.kUntilSecondMagnitude))
            .andThen(SecondMagnitude.raceWith(Commands.waitTime(Constants.Shooter.kUntilThirdMagnitude)))
            .andThen(ThirdMagnitude);

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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>()); // new SendableChooser<Command>()

    if (testing) {
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

      /** Test autonomous firing sequence. */
      autoChooser.addOption("Shooting Sequence", runFiringSequence);
    }

    // Secondary autonomous routine.
    // autoChooser.addOption("A-Bineutral Right", new PathPlannerAuto("A-Bineutral", false));
    // autoChooser.addOption("A-Bineutral Left", new PathPlannerAuto("A-Bineutral", true));

    // Primary autonomous routine.
    autoChooser.addOption("A-Unineutral Right", new PathPlannerAuto("A-Unineutral", false));
    autoChooser.addOption("A-Unineutral Left", new PathPlannerAuto("A-Unineutral", true));
    
    autoChooser.addOption("A-Short-Unineutral Right", new PathPlannerAuto("A-Short-Unineutral", false));
    autoChooser.addOption("A-Short-Unineutral Left", new PathPlannerAuto("A-Short-Unineutral", true));

    
    // Tertiary autonomous routine.
    autoChooser.addOption("CS-Bineutral", new PathPlannerAuto("CS-Bineutral"));

    autoChooser.addOption("A-Shoot-Depot", new PathPlannerAuto("A-Shoot-Depot"));
    autoChooser.addOption("A-Back-Up", new PathPlannerAuto("A-Back-Up"));
  }

  /**
   * Flywheel shot for this cycle, from the current odometry pose and the fixed hub pose.
   *
   * <p>Robot translation is {@link Drive#getPose()} with no vision blend, held pose, or lookahead.
   * The hub translation is {@link Constants.Poses#hub} and is not filtered. Chassis velocity
   * enters the fixed-hood projectile equation. {@link #calculateHubRotation()} holds that aim.
   */
  private HubShot.Solution currentShot() {
    try {
      Pose2d pose = drive.getPose();
      Pose2d hub = Constants.Poses.hub.getPose();
      if (!HubShot.isFinite(pose) || !HubShot.isFinite(hub)) {
        return HubShot.fallback();
      }
      ChassisSpeeds field = drive.getFieldVelocity();
      if (!HubShot.isFinite(field)) {
        field = new ChassisSpeeds();
      }

      HubShot.Input input = new HubShot.Input();
      input.robot = pose.getTranslation();
      input.hub = hub.getTranslation();
      input.vxMetersPerSecond = field.vxMetersPerSecond;
      input.vyMetersPerSecond = field.vyMetersPerSecond;
      input.axMetersPerSecondSquared = 0.0;
      input.ayMetersPerSecondSquared = 0.0;
      input.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
      // Put the muzzle on the hub line, not on whatever way the chassis is facing right now.
      // Otherwise the aim swings as the robot rotates toward the shot.
      double dx = hub.getX() - pose.getX();
      double dy = hub.getY() - pose.getY();
      if (dx * dx + dy * dy > 1e-6) {
        input.headingRadians = Math.atan2(dy, dx) + Math.PI;
      } else {
        input.headingRadians = pose.getRotation().getRadians();
      }
      input.shooterForwardMeters = Constants.Shooter.kShooterForwardMeters;
      input.shooterLeftMeters = Constants.Shooter.kShooterLeftMeters;
      input.lookaheadSeconds = 0.0;
      input.hoodPitchRadians = Math.toRadians(Dashboard.hoodPitch.get(1.0, 89.0));
      input.launchHeightMeters = Constants.Shooter.kShooterHeightMeters;
      input.hubEntryHeightMeters = Constants.Shooter.kHubEntryHeightMeters;
      input.gravityMetersPerSecondSquared = Constants.Shooter.kGravityMetersPerSecondSquared;
      input.maxFieldSpeed = Constants.Shooter.kMaxFieldSpeedMetersPerSecond;
      input.maxRpm = Constants.Shooter.kMaxRpm;
      input.regressionBase = Constants.base;
      input.regressionExp = Constants.exponential;
      input.rpmForDistance = Constants::regressRaw;

      HubShot.Solution solved = HubShot.solve(input);
      logShot(solved);
      return solved;
    } catch (RuntimeException ex) {
      Logger.recordOutput("Align/Fault", ex.toString());
      return HubShot.fallback();
    }
  }

  /** Publish the flywheel solution. Heading is logged separately from odometry. */
  private void logShot(HubShot.Solution shot) {
    Translation2d release = HubShot.isFinite(shot.pose) ? shot.pose : Translation2d.kZero;
    Rotation2d travel = shot.aim == null ? Rotation2d.kZero : shot.aim;
    // RPM solution only. The commanded heading is logged from calculateHubRotation.
    Logger.recordOutput("Align/Release", new Pose2d(release, travel));
    Logger.recordOutput("Align/ShotAim", travel);
    Logger.recordOutput("Align/Lead", shot.leadRadians);
    Logger.recordOutput("Align/PerpVelocity", shot.perpMetersPerSecond);
    Logger.recordOutput("Align/RadialVelocity", shot.radialMetersPerSecond);
    Logger.recordOutput("Align/FlightSeconds", shot.flightSeconds);
    Logger.recordOutput("Align/Live", shot.live);
    Logger.recordOutput("Shooter/VelocityMode", velocityType.name());
    Logger.recordOutput("Shooter/RpmStationary", shot.stationaryRpm);
    Logger.recordOutput("Shooter/DistanceMeters", shot.distanceMeters);
    Logger.recordOutput("Shooter/EffectiveDistance", shot.effectiveDistanceMeters);
    Logger.recordOutput("Shooter/CommandedRPM", shot.rpm);
  }

  /**
   * Hopper may feed. Outside shoot-on-the-fly this is only {@link Shooter#isReady()}. While
   * operator X is held, heading must also be inside {@link Constants.Shooter#kAlignmentError}
   * unless the dashboard alignment requirement is turned off.
   */
  private boolean feedAllowed() {
    try {
      if (!shootOnFly) {
        return shooter.isReady();
      }
      boolean headingOk = aligned.getAsBoolean() || !Dashboard.alignmentRequirement.get();
      return headingOk && shooter.isReady();
    } catch (RuntimeException ex) {
      return false;
    }
  }

  /**
   * Drop shoot-on-the-fly if the aim command ends, is interrupted, or the robot disables. Only
   * clears regression when this mode set it, so autonomous {@code AUTO} velocity is left alone.
   */
  public void releaseShootOnFly() {
    shootOnFly = false;
    if (velocityType == VelocityType.REGRESSION) {
      velocityType = VelocityType.STATIC;
    }
  }

  /**
   * Chassis heading for the shot. The hub pose is fixed and the robot pose is odometry. The held
   * heading is the velocity-compensated shot direction plus 180°, because the muzzle faces
   * backward. If the shot cannot be solved, the target stays on the current gyro heading.
   */
  private Rotation2d calculateHubRotation() {
    Pose2d robot = drive.getPose();
    Pose2d hub = Constants.Poses.hub.getPose();
    Rotation2d measured = drive.getRotation();
    if (measured == null || !Double.isFinite(measured.getRadians())) {
      measured = Rotation2d.kZero;
    }

    HubShot.Solution shot = currentShot();
    Rotation2d aim = measured;
    if (shot.live && shot.aim != null && Double.isFinite(shot.aim.getRadians())) {
      aim = shot.aim.plus(Rotation2d.k180deg);
    }
    drive.setRotationTarget(aim);

    double error = aim.minus(measured).getDegrees();
    if (!Double.isFinite(error)) {
      error = 180.0;
    }
    if (HubShot.isFinite(robot)) {
      Logger.recordOutput("Hub Pointer", new Pose2d(robot.getTranslation(), aim));
    }
    Logger.recordOutput("Align/Hub", hub);
    Logger.recordOutput("Align/Target", aim);
    Logger.recordOutput("Align/Measured", measured);
    Logger.recordOutput("Align/Error", error);
    return aim;
  }

  private Rotation2d calculatePassingRotation() {
    // Get poses.
    Pose2d robotPose = drive.getPose();
    Pose2d pointer = Constants.Poses.pointer.getPose();

    // pointer = (drive.isRightSide()) ? pointer : Constants.mirror(pointer);
    
    // Pose differences.
    double dx = pointer.getX() - robotPose.getX();
    double dy = pointer.getY() - robotPose.getY();

    // Angle from robot to hub
    Angle toPass = (Radians.of(Math.IEEEremainder(Math.atan2(dy, dx), Constants.Mathematics.TAU)));
    
    // Update drive target.
    drive.setRotationTarget(new Rotation2d(toPass));

    return drive.getRotationTarget();
  }

  /** Orient robot to face the hub. */
  private Command firingOrientation() {
    // return drive.isNeutralZone() 
    //   ? pointToAngle(this::calculatePassingRotation)
    //   : pointToAngle(this::calculateHubRotation);
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

    // Hold wheel position. Requires the drive so it cancels hub aim while held.
    Constants.Joysticks.driver
        .rightBumper()
        .whileTrue(Commands.run(drive::stopWithX, drive));

    // Zero pose heading.
    Constants.Joysticks.driver
        .a()
        .onTrue(Commands.runOnce(
          () -> drive.setPose(new Pose2d(drive.getPose()
              .getTranslation(), Rotation2d.kZero)), drive)
              .ignoringDisable(true));

    // Toggle intake between raised and lowered positions to aggitate fuel.
    Constants.Joysticks.operator
        .povUp()
        .onTrue(intake.raiseWrist(Degrees.of(60)).alongWith(intake.run().repeatedly()))
        .whileFalse(intake.lowerWrist().alongWith(intake.halt()));

    // Run intake rollers at full speed when left trigger is held, and halt when released.
    Constants.Joysticks.operator.leftTrigger()
        .onTrue(intake.run().repeatedly())
        .onFalse(intake.halt());

    // Run shooter at target velocity when right bumper is held, and halt when released.
    Constants.Joysticks.operator
        .rightTrigger()
        .whileTrue(
          shooter.run().repeatedly().alongWith(Commands.either(
            hopper.run(), hopper.halt(), this::feedAllowed).repeatedly()))
        .onFalse(
          shooter.halt().alongWith(hopper.halt()));

    Constants.Joysticks.operator
        .rightBumper()
        .whileTrue(
          shooter.run().repeatedly())
        .onFalse(
          shooter.halt());

    // Run feeding mechanism.
    Constants.Joysticks.operator
        .leftBumper()
        .whileFalse(hopper.halt())
        .whileTrue(
            Commands.either(hopper.run(), hopper.halt(), () -> !shootOnFly || feedAllowed())
                .repeatedly());

    // Raise intake to avoid impact.
    // Constants.Joysticks.operator.povRight().onFalse(intake.lowerWrist()).onTrue(intake.emergency());

    // Invert all control.
    Constants.Joysticks.operator.povLeft().whileTrue(invertion(true)).whileFalse(invertion(false));

    // Hold X: translate with the left stick, hold the back of the robot on the hub, live RPM.
    Constants.Joysticks.operator
        .x()
        .whileTrue(
            firingOrientation()
                .beforeStarting(
                    () -> {
                      shootOnFly = true;
                      velocityType = VelocityType.REGRESSION;
                    })
                .finallyDo(this::releaseShootOnFly));
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
