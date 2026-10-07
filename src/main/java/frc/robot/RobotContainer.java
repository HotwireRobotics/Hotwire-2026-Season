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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.indication.limelights.LimelightArray.IMUMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.simulation.FuelSim;
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

  /** Fuel physics. Null on the real robot and during log replay. */
  public final FuelSim fuelSim;

  // Static configuration.
  private final boolean firstPerson = false;
  private final boolean testing = true;

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

  /** Last finite odometry pose, used if the estimator returns NaN. */
  private Pose2d lastFinitePose = new Pose2d();

  /** Last live shot, held if a cycle cannot be solved. */
  private HubShot.Solution lastSolution = HubShot.fallback();

  /** Smooths module-speed noise before it moves the virtual hub. */
  private final HubShot.VelocityFilter shotVelocity = new HubShot.VelocityFilter();

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
            // Measure.isNear compares raw magnitudes and does not wrap. 179° and -179°
            // would look 358° apart and the hopper would never feed while aimed.
            return HubShot.headingsAligned(
                measured, target, Constants.Shooter.kAlignmentError.in(Radians));
          } catch (RuntimeException ex) {
            return false;
          }
        };

    // Velocity supplier.
    velocity =
        () -> {
          switch (velocityType) {
            case STATIC:
              // Ferrying static velocity.
              return Constants.Shooter.kSpeed;
            case REGRESSION:
            case AUTO:
              // Recomputed on every read: distance, closing speed, and sideways speed.
              return RPM.of(currentShot().rpm);
            case TESTING:
              // Allow testing of shooter velocity via dashboard input, for characterization purposes.
              return RPM.of(SmartDashboard.getNumber("Test Shooter RPM", testVelocity));
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

    // Fuel sim reads the same muzzle constants as HubShot and adds field velocity to each launch.
    fuelSim =
        Constants.currentMode == Constants.Mode.SIM
            ? new FuelSim(
                drive::getPose,
                drive::getFieldVelocity,
                shooter::isFiring,
                intake::isIntaking,
                velocity)
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
    autoChooser.addOption("A-Bineutral Right", new PathPlannerAuto("A-Bineutral", false));
    autoChooser.addOption("A-Bineutral Left", new PathPlannerAuto("A-Bineutral", true));

    // Primary autonomous routine.
    autoChooser.addOption("A-Unineutral Right", new PathPlannerAuto("A-Unineutral", false));
    autoChooser.addOption("A-Unineutral Left", new PathPlannerAuto("A-Unineutral", true));

    // Tertiary autonomous routine.
    // autoChooser.addOption("A-Depot", new PathPlannerAuto("A-Depot"));
    autoChooser.addOption("A-Shoot-Depot", new PathPlannerAuto("A-Shoot-Depot"));
  }

  /**
   * Pose used to aim. Odometry by default. A fresh Limelight translation is used only when vision
   * is enabled, the estimate is young, and it agrees with odometry. The heading always stays the
   * gyro heading.
   */
  private Pose2d aimPose() {
    Pose2d odometry = drive.getPose();
    if (!HubShot.isFinite(odometry)) {
      odometry = lastFinitePose;
    } else {
      lastFinitePose = odometry;
    }
    if (vision == null || !Dashboard.visionEnabled.get()) {
      return odometry;
    }
    Pose2d visionPose = vision.getFreshPose(Constants.Shooter.kVisionMaxAgeSeconds);
    if (!HubShot.isFinite(visionPose)) {
      return odometry;
    }
    double disagreement =
        visionPose.getTranslation().getDistance(odometry.getTranslation());
    if (!Double.isFinite(disagreement)
        || disagreement > Constants.Shooter.kVisionMaxDisagreementMeters) {
      return odometry;
    }
    return new Pose2d(visionPose.getTranslation(), odometry.getRotation());
  }

  /**
   * Shot for this cycle. Chassis velocity changes both the aim point and the flywheel RPM. A bad
   * sample holds the previous live shot.
   */
  private HubShot.Solution currentShot() {
    try {
      Pose2d pose = aimPose();
      Pose2d hub = Constants.Poses.hub.getPose();
      if (!HubShot.isFinite(pose) || !HubShot.isFinite(hub)) {
        return lastSolution;
      }
      ChassisSpeeds field = drive.getFieldVelocity();
      if (!HubShot.isFinite(field)) {
        field = new ChassisSpeeds();
      }
      Translation2d filtered =
          shotVelocity.update(
              field.vxMetersPerSecond,
              field.vyMetersPerSecond,
              Timer.getFPGATimestamp(),
              Dashboard.velocityFilter.get(0.0, 0.25));
      Translation2d accel = shotVelocity.acceleration();

      HubShot.Input input = new HubShot.Input();
      input.robot = pose.getTranslation();
      input.hub = hub.getTranslation();
      input.vxMetersPerSecond = filtered.getX();
      input.vyMetersPerSecond = filtered.getY();
      input.axMetersPerSecondSquared = accel.getX();
      input.ayMetersPerSecondSquared = accel.getY();
      input.omegaRadiansPerSecond = field.omegaRadiansPerSecond;
      input.headingRadians = pose.getRotation().getRadians();
      input.shooterForwardMeters = Constants.Shooter.kShooterForwardMeters;
      input.shooterLeftMeters = Constants.Shooter.kShooterLeftMeters;
      input.lookaheadSeconds = Dashboard.shotLookahead.get(0.0, 0.40);
      input.metersPerSecondPerRpm = Dashboard.exitSpeedPerRpm.get(0.001, 0.02);
      input.hoodPitchRadians = Math.toRadians(Dashboard.hoodPitch.get(20.0, 75.0));
      input.leadGainRadiansPerMps = Dashboard.leadGain.get(-0.20, 0.20);
      input.minFlightSeconds = Constants.Shooter.kMinFlightSeconds;
      input.maxFlightSeconds = Constants.Shooter.kMaxFlightSeconds;
      input.minScale = Constants.Shooter.kMinRpmScale;
      input.maxScale = Constants.Shooter.kMaxRpmScale;
      input.maxLeadRadians = Math.toRadians(Constants.Shooter.kMaxLeadDegrees);
      input.maxFieldSpeed = Constants.Shooter.kMaxFieldSpeedMetersPerSecond;
      input.minExitMetersPerSecond = Constants.Shooter.kMinExitMetersPerSecond;
      input.minDistanceMeters = Constants.Shooter.kMinShotMeters;
      input.maxDistanceMeters = Constants.Shooter.kMaxShotMeters;
      input.maxRpm = Constants.Shooter.kMaxRpm;
      input.regressionBase = Constants.base;
      input.regressionExp = Constants.exponential;
      input.rpmForDistance = Constants::regressRaw;

      HubShot.Solution solved = HubShot.solve(input);
      if (!solved.live && lastSolution.live) {
        solved = lastSolution;
      } else if (solved.live) {
        lastSolution = solved;
      }
      logShot(solved);
      return solved;
    } catch (RuntimeException ex) {
      Logger.recordOutput("Align/Fault", ex.toString());
      return lastSolution;
    }
  }

  /** Publish the shot that aim and RPM are both using. */
  private void logShot(HubShot.Solution shot) {
    Translation2d pose = HubShot.isFinite(shot.pose) ? shot.pose : Translation2d.kZero;
    Rotation2d aim = shot.aim == null ? Rotation2d.kZero : shot.aim;
    Logger.recordOutput("Hub Pointer", new Pose2d(pose, aim));
    Logger.recordOutput("Align/Target", aim);
    Logger.recordOutput("Align/Lead", shot.leadRadians);
    Logger.recordOutput("Align/PerpVelocity", shot.perpMetersPerSecond);
    Logger.recordOutput("Align/RadialVelocity", shot.radialMetersPerSecond);
    Logger.recordOutput("Align/FlightSeconds", shot.flightSeconds);
    Logger.recordOutput("Align/Live", shot.live);
    Logger.recordOutput("Shooter/VelocityMode", velocityType.name());
    Logger.recordOutput("Shooter/RpmStationary", shot.stationaryRpm);
    Logger.recordOutput("Shooter/DistanceMeters", shot.distanceMeters);
    Logger.recordOutput("Shooter/EffectiveDistance", shot.effectiveDistanceMeters);
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
   * Field heading that keeps the chassis on the moving hub shot. Called every drive cycle while
   * operator X is held. Right-stick rotate is not part of this command.
   */
  private Rotation2d calculateHubRotation() {
    HubShot.Solution shot = currentShot();
    Rotation2d aim = shot.aim == null ? Rotation2d.kZero : shot.aim;
    drive.setRotationTarget(aim);

    Rotation2d measured = drive.getRotation();
    if (measured == null || !Double.isFinite(measured.getRadians())) {
      measured = Rotation2d.kZero;
    }
    double error = aim.minus(measured).getDegrees();
    if (!Double.isFinite(error)) {
      error = 180.0;
    }
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

    // Hold wheel position.
    Constants.Joysticks.driver.rightBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
        .onFalse(intake.lowerWrist().alongWith(intake.halt()));

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

    // Manual feed. While shoot-on-the-fly is held, the same ready gate as right trigger applies.
    Constants.Joysticks.operator
        .leftBumper()
        .whileFalse(hopper.halt())
        .whileTrue(
            Commands.either(hopper.run(), hopper.halt(), () -> !shootOnFly || feedAllowed())
                .repeatedly());

    // Raise intake to avoid impact.
    Constants.Joysticks.operator.povRight().onFalse(intake.lowerWrist()).onTrue(intake.emergency());

    // Invert all control.
    Constants.Joysticks.operator.povLeft().whileTrue(invertion(true)).whileFalse(invertion(false));

    // Hold operator X: left stick translates, heading and RPM track the moving hub shot.
    // Right-stick rotate is not in this command. Releasing X, or any interrupt, clears the mode.
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
