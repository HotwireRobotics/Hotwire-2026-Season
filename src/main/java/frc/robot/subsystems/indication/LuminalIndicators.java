package frc.robot.subsystems.indication;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TimeTrigger;

public class LuminalIndicators extends SubsystemBase {

  private final CANdle candle;

  public LuminalIndicators() {
    candle = new CANdle(0);
  }

  @Override
  public void periodic() {
    Time t = Seconds.of(DriverStation.getMatchTime());
    Time length = (DriverStation.isAutonomous()) ? Constants.autoLength : Constants.teleopLength;

    // Get period-relative time.
    Time time = (t.isEquivalent(Seconds.of(-1))) ? Seconds.of(0) : length.minus(t);

    for (TimeTrigger trigger :
        ((DriverStation.isAutonomous())
            ? Constants.Indication.Autonomous.times
            : Constants.Indication.Teloperated.times)) {
      if (trigger.isTriggered(time)) {
        updateLEDs(trigger.getColor());
      }
    }
  }

  public Command updateLEDs(SolidColor color) {
    return runOnce(
        () -> {
          candle.setControl(color);
        });
  }
}
