package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Handles hopper commands and telemetry through an IO abstraction layer. */
public class Hopper extends SubsystemBase implements Systerface {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private enum State {
    STOPPED,
    RUNNING
  }

  private State state = State.STOPPED;
  private double commandedOutput = 0.0;

  /** Creates a hopper subsystem with a specific IO backend. */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  /** Creates a hopper subsystem backed by the default real hardware IO. */
  public Hopper() {
    this(new HopperIOTalonFX());
  }

  /** Returns the current high-level hopper state. */
  public Object getState() {
    return state;
  }

  /** Polls IO inputs and publishes hopper telemetry each loop. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    Logger.recordOutput("Hopper/State", state.toString());
    Logger.recordOutput("Hopper/CommandedOutput", commandedOutput);
  }

  /** Runs the hopper forward using the configured default speed. */
  public Command runHopper() {
    return runHopper(() -> false);
  }

  /** Runs the hopper and optionally inverts direction for driver/operator controls. */
  public Command runHopper(Supplier<Boolean> inverse) {
    return Commands.runOnce(
        () -> {
          commandedOutput = Constants.Hopper.kSpeed * (inverse.get() ? -1.0 : 1.0);
          io.setHopperOpenLoop(commandedOutput);
          state = commandedOutput == 0.0 ? State.STOPPED : State.RUNNING;
        },
        this);
  }

  /** Stops hopper motion and updates subsystem state. */
  public Command stopHopper() {
    return Commands.runOnce(
        () -> {
          commandedOutput = 0.0;
          io.setHopperOpenLoop(0.0);
          state = State.STOPPED;
        },
        this);
  }
}
