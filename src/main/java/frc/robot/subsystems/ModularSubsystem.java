package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

public class ModularSubsystem extends SubsystemBase {
  private final HashMap<Object, Object> devices = new HashMap<Object, Object>();
  private List<Object> active = new ArrayList<Object>();

  public void specifyActiveDevice(Object device) {
    for (Motor d : getDevices(device)) {
      active.add(d);
    }
  }

  public void specifyInactiveDevice(Object device) {
    for (Motor d : getDevices(device)) {
      active.remove(d);
    }
  }

  public boolean isActiveDevice(Object device) {
    boolean isActive = false;
    for (Motor d : getDevices(device)) {
      isActive = isActive || active.contains(d);
    }
    return isActive;
  }

  public void defineDevice(Object device, Motor actual) {
    devices.put(device, actual);
  }

  public void defineDevice(Object device, Motor[] actual) {
    devices.put(device, actual);
  }

  public Command runDevice(Object device, double speed) {
    return Commands.runOnce(
        () -> {
          for (Motor d : getDevices(device)) {
            d.set(speed);
          }
          if (speed == 0) {
            specifyInactiveDevice(device);
          } else {
            specifyActiveDevice(device);
          }
        });
  }

  public Command runDevice(Object device, double speed, Subsystem subsystem) {
    Command command = runDevice(device, speed);
    command.addRequirements(subsystem);
    return command;
  }

  public Command runDevice(Object device, Supplier<Double> speed) {
    return Commands.runOnce(
        () -> {
          for (Motor d : getDevices(device)) {
            d.set(speed.get());
          }
          if (speed.get() == 0) {
            specifyInactiveDevice(device);
          } else {
            specifyActiveDevice(device);
          }
        });
  }

  public Command runDevice(Object device, Supplier<Double> speed, Subsystem subsystem) {
    Command command = runDevice(device, speed);
    command.addRequirements(subsystem);
    return command;
  }

  public class DevicePointer {
    private Object device;
    private Object actual;

    public DevicePointer(Object device, Motor actual) {
      this.device = device;
      this.actual = actual;
    }

    public DevicePointer(Object device, Motor... actual) {
      this.device = device;
      this.actual = actual;
    }

    public Object getDevice() {
      return device;
    }

    public Object getActual() {
      return actual;
    }
  }

  public void defineDevice(DevicePointer... pointers) {
    for (DevicePointer pointer : pointers) {
      devices.put(pointer.getDevice(), pointer.getActual());
    }
  }

  public void defineDevice(DevicePointer pointer) {
    devices.put(pointer.getDevice(), pointer.getActual());
  }

  public Motor[] getDevices(Object device) {
    Object group = devices.get(device);
    if (group instanceof Motor[]) {
      return ((Motor[]) group);
    } else {
      Motor[] items = {(Motor) group};
      return items;
    }
  }

  public void logDevices() {
    for (Object device : devices.keySet()) {
      for (Motor d : getDevices(device)) {
        String name = device.toString();
        Logs.log(name, d);
      }
    }
  }

  private void applyPercent(double percent, Motor... motors) {
    for (var m : motors) m.set(percent);
  }
}
