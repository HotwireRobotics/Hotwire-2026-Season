package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.Feedforward;
import frc.robot.subsystems.motors.TalonFXIO;
import frc.robot.subsystems.motors.MotorBase.Direction;
import frc.robot.subsystems.motors.MotorBase.FollowerMode;
import frc.robot.subsystems.motors.MotorBase.NeutralMode;

import java.util.function.Supplier;

public class Intake extends ModularSubsystem implements Systerface {

  // Declare devices.
  public final Motor rollers, follower, wrist;

  // Declare suppliers.
  private final Supplier<Double> speed;

  // Declare device enum.
  public enum Device {
    ROLLERS,
    RIGHT,
    LEFT,
  }

  public enum ArmState {
    FORWARD,
    BACKWARD,
    ZERO
  }

  private ArmState armState = ArmState.ZERO;

  public Intake(Supplier<Double> speed) {
    // Initialize devices.
    rollers = new Motor(this, new TalonFXIO(Constants.MotorIDs.i_rollers));
    rollers.setCurrentLimit(Amps.of(40));
    rollers.setDirection(Direction.FORWARD);
    rollers.setNeutralMode(NeutralMode.COAST);

    follower = new Motor(this, new TalonFXIO(Constants.MotorIDs.i_wristL));
    follower.setCurrentLimit(Amps.of(45));
    follower.setDirection(Direction.REVERSE);
    follower.setNeutralMode(NeutralMode.COAST);

    wrist = new Motor(this, new TalonFXIO(Constants.MotorIDs.i_wristR));
    wrist.setCurrentLimit(Amps.of(45));
    wrist.setDirection(Direction.FORWARD);
    wrist.setNeutralMode(NeutralMode.COAST);
    
    // Define devices.
    defineDevice(
      new DevicePointer(Device.ROLLERS, rollers),
      new DevicePointer(Device.LEFT, follower),
      new DevicePointer(Device.RIGHT, wrist)
    );

    // Initialize control loop.
    wrist.apply(new Feedforward(14, 0, 0));

    follower.follow(wrist, FollowerMode.INVERSE);

    this.speed = speed;
  }

  public Intake() {
    this(() -> Constants.Intake.kSpeed);
  }

  // State system.
  private enum State {
    STOPPED,
    INTAKING
  }

  State state = State.STOPPED;

  // Supply state.
  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    // Log devices and state.
    logDevices();
    
    Logs.log(this, state);
    Logs.write("Intake/ArmState", armState);

    if (isActiveDevice(Device.ROLLERS)) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
  }

  /** Run the hopper at the specified speed. */
  public Command run() {
    return runDevice(Device.ROLLERS, speed, this);
  }

  /** Halt the hopper. */
  public Command halt() {
    return runDevice(Device.ROLLERS, 0, this);
  }

  public Command oscillateArm(Angle angle, Frequency frequency) {
    Time period = Seconds.of(1 / (2 * frequency.in(Hertz)));
    RepeatCommand group =
        new SequentialCommandGroup(
                lowerWrist(), Commands.waitTime(period),
                raiseWrist(angle), Commands.waitTime(period))
            .repeatedly();
    group.addRequirements(this);
    return group;
  }

  public Command raiseWrist(Angle angle) {
    return Commands.runOnce(
        () -> wrist.runPosition(angle));
  }

  public Command lowerWrist() {
    return Commands.runOnce(
        () -> wrist.runPosition(Degrees.of(0)));
  }

  public Command emergency() {
    return Commands.runOnce(
        () -> wrist.runPosition(Degrees.of(90)));
  }
}
