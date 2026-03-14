package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.subsystems.Motor;
import java.util.function.Supplier;

public class Intake extends ModularSubsystem implements Systerface {

  // Declare devices.
  public final Motor rollers, wrist;

  // Declare suppliers.
  private final Supplier<Double> speed;

  // Declare control loop.
  private final PositionVoltage control;
  private final Slot0Configs slot;

  // Declare device enum.
  public enum Device {
    ROLLERS,
    WRIST
  }

  public enum ArmState {
    FORWARD,
    BACKWARD,
    ZERO
  }

  private ArmState armState = ArmState.ZERO;

  public Intake(Supplier<Double> speed) {
    // Initialize devices.
    rollers = new Motor(this, Constants.MotorIDs.i_rollers, Amps.of(40));
    wrist = new Motor(this, Constants.MotorIDs.i_arm, Amps.of(40));

    // Define devices.
    defineDevice(
        new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.WRIST, wrist));

    // Initialize control loop.
    control = new PositionVoltage(Degrees.of(0));

    slot = new Slot0Configs();
    configureProportional(18.0);

    // Configuration
    wrist.setControl(control);
    wrist.getConfigurator().apply(slot);

    this.speed = speed;
  }

  public Intake() {
    this(() -> Constants.Intake.kSpeed);
  }

  public void configureProportional(double kP) {
    slot.withKP(kP);
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
    rollers.log();
    wrist.log();
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
        () -> {
          configureProportional(18.0);
          wrist.setControl(control.withPosition(angle));
        });
  }

  public Command lowerWrist() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          wrist.setControl(control.withPosition(Degrees.of(0)));
        });
  }

  public Command emergency() {
    return Commands.runOnce(
        () -> {
          configureProportional(28.0);
          wrist.setControl(control.withPosition(Degrees.of(90)));
        });
  }
}
