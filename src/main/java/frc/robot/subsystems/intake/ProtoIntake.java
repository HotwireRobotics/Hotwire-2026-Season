package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem. All hardware interaction is via IntakeIO so inputs are loggable and replayable.
 */
public class ProtoIntake extends SubsystemBase implements Systerface {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Commanded outputs for state derivation (no hardware in subsystem). */
  private double lastRollersOutput = 0.0;

  private double lastLowerOutput = 0.0;

  public enum Device {
    ROLLERS,
    LOWER
  }

  public ProtoIntake(IntakeIO io) {
    this.io = io;
  }

  private enum State {
    STOPPED,
    INTAKING
  }

  State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", state.toString());

    if (lastRollersOutput != 0.0 || lastLowerOutput != 0.0) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
  }

  /** Run a device by enum at the given output (-1 to 1). */
  public void runDevice(Device device, double speed) {
    switch (device) {
      case ROLLERS -> {
        io.setRollersOutput(speed);
        lastRollersOutput = speed;
      }
      case LOWER -> {
        io.setLowerOutput(speed);
        lastLowerOutput = speed;
      }
    }
  }

  /**
   * For ModularSubsystem compatibility: no TalonFX in subsystem, so always false for "active" by
   * device.
   */
  public boolean isActiveDevice(Device device) {
    return switch (device) {
      case ROLLERS -> lastRollersOutput != 0.0;
      case LOWER -> lastLowerOutput != 0.0;
    };
  }

  public Command runMechanism(double speed) {
    return Commands.run(
        () -> {
          runDevice(Device.ROLLERS, speed);
          runDevice(Device.LOWER, speed);
        });
  }
}
