package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Systerface;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase implements Systerface {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private double upperSpeedtest;
  private double lowerSpeedtest;

  // private final SysIdRoutine m_sysIdRoutine;
  // private final VoltageOut m_voltReq;
  // private final VelocityVoltage m_velVolt;

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

  // public void runLower(double speed) {
  //   lowerFeed.set(speed);
  // }
}
