package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class HopperSubsystem extends ModularSubsystem implements Systerface {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public enum Device {
    HOPPER
  }

  /** Constructs hopper with chosen IO implementation. */
  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  private enum State {
    STOPPED
  }

  private State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    Logger.recordOutput("Hopper/State", state.toString());
    // Logger.recordOutput(
    //     "Hopper/lowerFeed/Position", lowerFeed.getPosition().getValueAsDouble(), "rot");
    // Logger.recordOutput(
    //     "Hopper/lowerFeed/Velocity", lowerFeed.getVelocity().getValueAsDouble() * 60, "rpm");
    // Logger.recordOutput(
    //     "Hopper/lowerFeed/Voltage", lowerFeed.getMotorVoltage().getValueAsDouble(), "V");
    // Logger.recordOutput(
    //     "Hopper/lowerFeed/Current", lowerFeed.getSupplyCurrent().getValueAsDouble(), "A");
    // Logger.recordOutput(
    //     "Hopper/lowerFeed/Temperature", lowerFeed.getDeviceTemp().getValueAsDouble(), "°C");
  }

  public Command runHopper(double speed) {
    return Commands.runOnce(() -> io.setUpperOpenLoop(speed));
  }

  public Command controlHopper(Supplier<Double> speed) {
    return Commands.runOnce(() -> io.setUpperOpenLoop(speed.get()), this);
  }

  public void runUpper(double speed) {
    io.setUpperOpenLoop(speed);
  }

  public Command stopHopper() {
    return runDevice(Device.HOPPER, 0);
  }
}
