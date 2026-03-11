package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends ModularSubsystem implements Systerface {
  private final TalonFX hopper;
public class HopperSubsystem extends SubsystemBase implements Systerface {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private double upperSpeedtest;
  private double lowerSpeedtest;

  public enum Device {
    HOPPER
  }

  public HopperSubsystem() {
    hopper = new TalonFX(Constants.MotorIDs.h_hopper);
    defineDevice(Device.HOPPER, hopper);
  /** Constructs hopper with chosen IO implementation. */
  public HopperSubsystem(HopperIO io) {
    this.io = io;
    upperSpeedtest = SmartDashboard.getNumber("upperSpeed", 0);
    lowerSpeedtest = SmartDashboard.getNumber("lowerSpeed", 0);
    SmartDashboard.putNumber("upperSpeed", upperSpeedtest);
    SmartDashboard.putNumber("lowerSpeed", lowerSpeedtest);

    // m_sysIdRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //         null,
    //         null,
    //         null, // Use default config
    //         (state) -> Logger.recordOutput("Hopper/SysIdTestState", state.toString()),
    //         new SysIdRoutine.Mechanism(
    //             null, //change
    //             null,
    //             this));
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
    
    Logger.recordOutput(
        "Hopper/upperFeed/Position", hopper.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Hopper/upperFeed/Velocity", hopper.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Hopper/upperFeed/Voltage", hopper.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Hopper/upperFeed/Current", hopper.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Hopper/upperFeed/Temperature", hopper.getDeviceTemp().getValueAsDouble(), "°C");
  }

  public Command runHopper() {
    return runDevice(Device.HOPPER, Constants.Hopper.kSpeed);
  }

  public Command stopHopper() {
    return runDevice(Device.HOPPER, 0);
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

  // public void runLower(double speed) {
  //   lowerFeed.set(speed);
  // }
}
