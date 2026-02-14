package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Hopper subsystem. All hardware interaction is via HopperIO so inputs are loggable and replayable.
 */
public class HopperSubsystem extends SubsystemBase implements Systerface {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  /** Logged dashboard inputs for runHoppertest so values are replayed. */
  private final LoggedNetworkNumber upperSpeedInput = new LoggedNetworkNumber("upperSpeed", 0.0);

  private final LoggedNetworkNumber lowerSpeedInput = new LoggedNetworkNumber("lowerSpeed", 0.0);

  public HopperSubsystem(HopperIO io) {
    this.io = io;
    SmartDashboard.putNumber("upperSpeed", 0);
    SmartDashboard.putNumber("lowerSpeed", 0);
  }

  private enum State {
    STOPPED
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    upperSpeedInput.periodic();
    lowerSpeedInput.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  /** Logged upper speed for runHoppertest (replay-safe). */
  public double getUpperSpeed() {
    return upperSpeedInput.get();
  }

  /** Logged lower speed for runHoppertest (replay-safe). */
  public double getLowerSpeed() {
    return lowerSpeedInput.get();
  }

  public Command runHopper(double speed) {
    return Commands.run(
        () -> {
          io.setUpperOutput(speed);
          io.setLowerOutput(speed);
        });
  }

  public Command runHoppertest() {
    return Commands.run(
        () -> {
          io.setUpperOutput(getUpperSpeed());
          io.setLowerOutput(getLowerSpeed());
        });
  }

  public void runUpper(double speed) {
    io.setUpperOutput(speed);
  }

  public void runLower(double speed) {
    io.setLowerOutput(speed);
  }
}
