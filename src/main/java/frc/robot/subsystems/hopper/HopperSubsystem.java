package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase implements Systerface {
  private final TalonFX upperFeed;
  private final TalonFX lowerFeed;
  private double upperSpeedtest;
  private double lowerSpeedtest;

  // private final SysIdRoutine m_sysIdRoutine;
  // private final VoltageOut m_voltReq;
  // private final VelocityVoltage m_velVolt;

  public HopperSubsystem() {
    upperFeed = new TalonFX(Constants.MotorIDs.h_upperFeed);
    lowerFeed = new TalonFX(Constants.MotorIDs.h_lowerFeed);
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

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Hopper/State", state.toString());
    Logger.recordOutput(
        "Hopper/upperFeed/Position", upperFeed.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Hopper/upperFeed/Velocity", upperFeed.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Hopper/upperFeed/Current", upperFeed.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Hopper/lowerFeed/Position", lowerFeed.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Hopper/lowerFeed/Velocity", lowerFeed.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Hopper/lowerFeed/Current", lowerFeed.getSupplyCurrent().getValueAsDouble(), "A");
  }

  public Command runHopper(double speed) {
    return Commands.run(
        () -> {
          upperFeed.set(speed);
          lowerFeed.set(speed);
        });
  }

  public Command runHoppertest() {
    return Commands.run(
        () -> {
          upperFeed.set(SmartDashboard.getNumber("upperSpeed", 0));
          lowerFeed.set(SmartDashboard.getNumber("lowerSpeed", 0));
        });
  }

  public void runUpper(double speed) {
    upperFeed.set(speed);
  }

  public void runLower(double speed) {
    lowerFeed.set(speed);
  }
}
