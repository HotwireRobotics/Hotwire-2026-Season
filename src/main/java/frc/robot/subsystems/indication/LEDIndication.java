package frc.robot.subsystems.indication;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDIndication extends SubsystemBase {

  private final CANdle candle;

  private enum Event {
    AUTOENABLED,
    ACTIVE,
    VISION,
    INACTIVE,
    TELEDISABLED,
    TELEENABLED,
    EMERGENCY,
    AUTODISABLED
  }

  private final HashMap<Event, Supplier<SolidColor>> color;
  private final Timer timer = new Timer();
  private Time time = Seconds.of(0);

  private Supplier<SolidColor> toggle(SolidColor t, SolidColor f, Frequency frequency) {
    return () -> (((Math.floor(time.in(Seconds) * frequency.in(Hertz)) % 2) == 1) ? t : f);
  }

  public LEDIndication() {
    candle = new CANdle(0);

    // Configuration
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;

    candle.getConfigurator().apply(config);

    color = new HashMap<>();
    color.put(Event.EMERGENCY, () -> Constants.Indication.LEDColor(180, 0, 0));
    color.put(Event.ACTIVE, () -> Constants.Indication.LEDColor(0, 180, 0));
    color.put(Event.INACTIVE, () -> Constants.Indication.LEDColor(180, 0, 0));
    color.put(
        Event.TELEDISABLED,
        toggle(
            Constants.Indication.LEDColor(180, 0, 0),
            Constants.Indication.LEDColor(0, 0, 0),
            Hertz.of(0.5)));
    color.put(
        Event.TELEENABLED,
        toggle(
            Constants.Indication.LEDColor(0, 255, 0),
            Constants.Indication.LEDColor(0, 0, 0),
            Hertz.of(3)));
    color.put(
        Event.VISION,
        toggle(
            Constants.Indication.LEDColor(100, 230, 100),
            Constants.Indication.LEDColor(0, 0, 0),
            Hertz.of(3)));
    color.put(Event.INACTIVE, () -> Constants.Indication.LEDColor(180, 0, 0));
    color.put(Event.EMERGENCY, () -> Constants.Indication.LEDColor(255, 0, 0));
    color.put(
        Event.AUTODISABLED,
        toggle(
            Constants.Indication.LEDColor(20, 20, 150),
            Constants.Indication.LEDColor(0, 0, 0),
            Hertz.of(0.5)));
    color.put(
        Event.AUTOENABLED,
        toggle(
            Constants.Indication.LEDColor(20, 20, 150),
            Constants.Indication.LEDColor(0, 0, 0),
            Hertz.of(3)));

    timer.start();
  }

  public void time() {
    timer.reset();
    timer.start();
  }

  @Override
  public void periodic() {
    Time t = Seconds.of(DriverStation.getMatchTime());
    Time length =
        (DriverStation.isAutonomous())
            ? Constants.Length.autonomous
            : Constants.Length.teleoperated;

    // Get period-relative time.
    time = (t.isEquivalent(Seconds.of(-1))) ? Seconds.of(timer.get()) : length.minus(t);

    indicatorPipeline(time);
  }

  private Supplier<SolidColor> getRequest(Event event) {
    return color.get(event);
  }

  public void indicatorPipeline(Time time) {
    Logger.recordOutput("Indicators/time", time);

    if (DriverStation.isEStopped()) {
      updateLEDs(getRequest(Event.EMERGENCY).get());
    }

    if (DriverStation.isAutonomous()) {
      if (DriverStation.isEnabled()) {
        updateLEDs(getRequest(Event.AUTOENABLED).get());
      } else {
        updateLEDs(getRequest(Event.AUTODISABLED).get());
      }
    } else {
      if (DriverStation.isEnabled()) {
        // if (Constants.Indication.autonomousVictory()) {
        //   updateLEDs(getRequest(Event.INACTIVE).get());
        // } else {
        //   updateLEDs(getRequest(Event.ACTIVE).get());
        // }
        updateLEDs(getRequest(Event.TELEENABLED).get());
      } else {
        updateLEDs(getRequest(Event.TELEDISABLED).get());
      }
    }
  }

  private void updateLEDs(ControlRequest color) {
    candle.setControl(color);
  }
}
