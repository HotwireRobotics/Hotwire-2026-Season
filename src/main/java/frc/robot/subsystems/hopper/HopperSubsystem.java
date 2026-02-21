package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Systerface;
import frc.robot.util.MotorTelemetry;

public class HopperSubsystem extends SubsystemBase implements Systerface {
  private final TalonFX upperFeed;
  private double upperSpeedtest;
  private double lowerSpeedtest;
  private final TalonFX lowerFeed;

  // private final SysIdRoutine m_sysIdRoutine;
  // private final VoltageOut m_voltReq;
  // private final VelocityVoltage m_velVolt;

  public HopperSubsystem() {
    upperFeed = new TalonFX(Constants.MotorIDs.h_upperFeed);
    lowerFeed = new TalonFX(Constants.MotorIDs.h_lowerFeed);
    SmartDashboard.putNumber("upperSpeed", 0.0);
    SmartDashboard.putNumber("lowerSpeed", 0.0);

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

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Hopper/State", state.toString());
    // Log position (rot), velocity (rpm), voltage, current, temp with unit metadata
    MotorTelemetry.logBasic("Hopper/upperFeed", upperFeed);
    MotorTelemetry.logBasic("Hopper/lowerFeed", lowerFeed);
  }

  /** Sets both hopper motors to the same percent output. */
  public void setHopperSpeed(double speed) {
    upperFeed.set(speed);
    lowerFeed.set(speed);
  }

  public Command runHopper(double speed) {
    return Commands.runOnce(() -> setHopperSpeed(speed));
  }

  /** Holds hopper speed while scheduled and stops on release. */
  public Command holdHopper(double speed) {
    return Commands.startEnd(() -> setHopperSpeed(speed), () -> setHopperSpeed(0.0), this);
  }
  // Hopper tests
  public Command runHoppertest() {
    return Commands.run(
        () -> {
          upperFeed.set(SmartDashboard.getNumber("upperSpeed", 0.0));
          lowerFeed.set(SmartDashboard.getNumber("lowerSpeed", 0.0));
        });
  }

  public void runUpper(double speed) {
    upperFeed.set(speed);
  }

  public void runLower(double speed) {
    lowerFeed.set(speed);
  }
}
