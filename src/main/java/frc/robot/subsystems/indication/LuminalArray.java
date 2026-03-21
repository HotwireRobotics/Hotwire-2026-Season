package frc.robot.subsystems.indication;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LuminalArray extends SubsystemBase {

  // Declare devices.
  private final CANdle candle;

  // Present available event markers.
  private enum Event {
    AUTOENABLED,
    ACTIVE,
    WANING,
    INACTIVE,
    WAXING,
    TELEDISABLED,
    EMERGENCY,
    VISION,
    AUTODISABLED,
    // WARNING
  }

  // Handle static and animation control requests.
  private static class Control {
    private final ControlRequest request;
    private final boolean clear;

    private Control(ControlRequest request, boolean clear) {
      this.request = request;
      this.clear = clear;
    }

    public void apply(CANdle candle) {
      if (clear) {
        candle.setControl(new EmptyAnimation(0));
      }
      candle.setControl(request);
    }

    public static Control of(ControlRequest request) {
      return new Control(request, true);
    }

    public static Control animation(ControlRequest request) {
      return new Control(request, false);
    }
  }

  // Predefined colors.
  private class Colors {
    public static final SolidColor DISABLED = Constants.Indication.LEDColor(180, 0, 0);
    public static final SolidColor ACTIVE = Constants.Indication.LEDColor(0, 180, 0);
    public static final SolidColor INACTIVE = Constants.Indication.LEDColor(0, 0, 0);
  }

  private class Controls {
    // Teloperated disabled.
    public Supplier<Control> red = () -> Control.of(Colors.DISABLED);

    // Hub state indicators.
    public Supplier<Control> active = () -> Control.of(Colors.ACTIVE);

    public Supplier<Control> waning =
        () -> Control.of(tick(Colors.ACTIVE, Colors.INACTIVE, Hertz.of(2)).get());

    public Supplier<Control> inactive = () -> Control.of(Colors.INACTIVE);

    public Supplier<Control> waxing =
        () -> Control.of(toggle(Colors.ACTIVE, Colors.INACTIVE, Hertz.of(2)).get());

    // public Supplier<Control> warning =
    //     () -> Control.of(chase(Colors.ACTIVE, Colors.INACTIVE, 7).get());

    // Teleoperated disabled.
    public Supplier<Control> teledisabled =
        () -> Control.of(Constants.Indication.LEDColor(255, 90, 0));

    // Indicate vision measurement to setup.
    public Supplier<Control> visible =
        () -> Control.of(Constants.Indication.LEDColor(255, 45, 100));

    // Autonomous disabled.
    public Supplier<Control> autodisabled = () -> Control.of(Colors.DISABLED);

    // Autonomous enabled.
    public Supplier<Control> autoenabled =
        () ->
            Control.animation(
                new FireAnimation(0, 55)
                    .withSlot(0)
                    .withBrightness(0.149)
                    .withDirection(AnimationDirectionValue.Forward)
                    .withSparking(0.6)
                    .withCooling(0.3)
                    .withFrameRate(Hertz.of(30)));
  }

  private final Controls controls = new Controls();

  private final Timer timer = new Timer();
  private Time time = Seconds.of(0);

  private Supplier<ControlRequest> toggle(ControlRequest t, ControlRequest f, Frequency frequency) {
    return () -> (((Math.floor(time.in(Seconds) * frequency.in(Hertz)) % 2) == 1) ? t : f);
  }

  private Supplier<ControlRequest> tick(
      ControlRequest high, ControlRequest low, Frequency frequency) {
    return () -> ((((time.in(Seconds) * frequency.in(Hertz)) % 2) >= 1.7) ? high : low);
  }

  // private Supplier<ControlRequest> chase(ControlRequest high, ControlRequest low, int wavelength)
  // {
  //   return () -> ((((time.in(Seconds) / wavelength) % 2) == 1) ? high : low);
  // };

  // Get the correct control request for the active event.
  private Supplier<Control> getRequest(Event event) {
    return switch (event) {
      case EMERGENCY -> controls.red;
      case ACTIVE -> controls.active;
      case WANING -> controls.waning;
      case INACTIVE -> controls.inactive;
      case WAXING -> controls.waxing;
      case TELEDISABLED -> controls.teledisabled;
      case VISION -> controls.visible;
      case AUTODISABLED -> controls.autodisabled;
      case AUTOENABLED -> controls.autoenabled;
        // case WARNING -> controls.warning;
    };
  }

  public LuminalArray() {
    candle = new CANdle(0);

    // Device configuration.
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;

    candle.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // Get relative time.
    time = Constants.Tempo.getTime();

    indicatorPipeline(time);
  }

  public void indicatorPipeline(Time time) {
    Logger.recordOutput("Indicators/time", time);
    Logger.recordOutput("Indicators/active", Constants.Indication.isActive());

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
        if (Constants.Indication.isActive()) {
          if (Constants.Indication.isWaning()) {
            updateLEDs(getRequest(Event.WANING).get());
          } else {
            updateLEDs(getRequest(Event.ACTIVE).get());
          }
        } else {
          if (Constants.Indication.isWaxing()) {
            updateLEDs(getRequest(Event.WAXING).get());
            // updateLEDs(getRequest(Event.WARNING).get());
          } else {
            updateLEDs(getRequest(Event.INACTIVE).get());
            // updateLEDs(getRequest(Event.WARNING).get());
          }
        }
      } else {
        updateLEDs(getRequest(Event.TELEDISABLED).get());
      }
    }
  }

  private Control state_ = null;

  private void updateLEDs(Control state) {
    if (state != state_) {
      state.apply(candle);
      state_ = state;
    }
  }
}
