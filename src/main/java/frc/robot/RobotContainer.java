package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final ProtoIntake intake;
  public final ProtoShooter shooter;

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
            () -> Constants.Joysticks.driver.getLeftY(),
            () -> Constants.Joysticks.driver.getLeftX(),
            () -> -Constants.Joysticks.driver.getRightX()));
    Constants.Joysticks.operator
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> Constants.Joysticks.driver.getLeftY(),
                () -> Constants.Joysticks.driver.getLeftX(),
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

    Constants.Joysticks.operator
        .a()
        .whileTrue(intake.runRollers(0.8))
        .onFalse(intake.runRollers(0));
    Constants.Joysticks.operator
        .rightBumper()
        .whileTrue(shooter.runShooterAndFeeder(1))
        .onFalse(shooter.runShooterAndFeeder(0));

    // Constants.Joysticks.driver
    //     .back()
    //     .onTrue(Commands.runOnce(() -> ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}


/**
 * Visualization for the elevator subsystem in simulation.
 */
public class ElevatorSubsystemSim extends SubsystemBase {

  private final ElevatorSubsystem elevator;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevatorMech;

  // Visualization constants
  private final double VISUAL_WIDTH = 10.0; // Width of visualization in pixels
  private final double BASE_HEIGHT = 20.0; // Height of base in pixels
  private final double CARRIAGE_WIDTH = 30.0; // Width of elevator carriage in pixels
  private final double CARRIAGE_HEIGHT = 40.0; // Height of elevator carriage in pixels

  // Elevator parameters
  private final double minHeight;
  private final double maxHeight;
  private final double visualScaleFactor;

  /**
   * Creates a new visualization for the elevator.
   *
   * @param elevatorSubsystem The elevator subsystem to visualize
   */
  public ElevatorSubsystemSim(ElevatorSubsystem elevatorSubsystem) {
    this.elevator = elevatorSubsystem;

    // Get elevator parameters from simulation
    minHeight = elevator.getMinHeightMeters();
    maxHeight = elevator.getMaxHeightMeters();

    // Calculate scale factor to keep visualization in reasonable bounds
    double elevatorTravel = maxHeight - minHeight;
    visualScaleFactor = 300.0 / elevatorTravel; // Scale to ~300 pixels max

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ElevatorRoot", 200, 50);

    // Add elevator base
    MechanismLigament2d elevatorBase = root.append(
      new MechanismLigament2d(
        "Base",
        BASE_HEIGHT,
        90,
        6,
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Add elevator tower
    MechanismLigament2d elevatorTower = elevatorBase.append(
      new MechanismLigament2d(
        "Tower",
        (maxHeight - minHeight) * visualScaleFactor,
        90,
        VISUAL_WIDTH,
        new Color8Bit(Color.kGray)
      )
    );

    // Add elevator carriage
    elevatorMech = root.append(
      new MechanismLigament2d(
        "Elevator",
        BASE_HEIGHT,
        90,
        CARRIAGE_WIDTH,
        new Color8Bit(Color.kBlue)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("Elevator Sim", mech);
  }

  @Override
  public void periodic() {
    // Update elevator height
    double currentHeight = elevator.getSimulation().getPositionMeters();
    double displayHeight =
      BASE_HEIGHT + (currentHeight - minHeight) * visualScaleFactor;
    elevatorMech.setLength(displayHeight);

    // Add telemetry data
    SmartDashboard.putNumber("Elevator Height (m)", currentHeight);
    SmartDashboard.putNumber(
      "Elevator Velocity (m/s)",
      elevator.getSimulation().getVelocityMetersPerSecond()
    );
    SmartDashboard.putNumber(
      "Elevator Current (A)",
      elevator.getSimulation().getCurrentDrawAmps()
    );
  }
}
