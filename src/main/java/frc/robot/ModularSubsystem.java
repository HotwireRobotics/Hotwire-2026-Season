package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ModularSubsystem<DeviceKey> extends SubsystemBase {
  private final Map<DeviceKey, TalonFX[]> devices = new HashMap<>();
  private final Set<TalonFX> active = new HashSet<>();

  /** Marks every motor in a logical device group as active. */
  public void specifyActiveDevice(DeviceKey device) {
    for (TalonFX d : getDevices(device)) {
      active.add(d);
    }
  }

  /** Marks every motor in a logical device group as inactive. */
  public void specifyInactiveDevice(DeviceKey device) {
    for (TalonFX d : getDevices(device)) {
      active.remove(d);
    }
  }

  /** Returns whether any motor in a logical device group is currently active. */
  public boolean isActiveDevice(DeviceKey device) {
    for (TalonFX d : getDevices(device)) {
      if (active.contains(d)) {
        return true;
      }
    }
    return false;
  }

  /** Associates one logical device key with one physical motor. */
  public void defineDevice(DeviceKey device, TalonFX actual) {
    devices.put(device, new TalonFX[] {actual});
  }

  /** Associates one logical device key with a motor group. */
  public void defineDevice(DeviceKey device, TalonFX[] actual) {
    devices.put(device, Arrays.copyOf(actual, actual.length));
  }

  /** Lightweight pointer used to batch device definitions. */
  public final class DevicePointer {
    private final DeviceKey device;
    private final TalonFX[] actual;

    public DevicePointer(DeviceKey device, TalonFX actual) {
      this.device = device;
      this.actual = new TalonFX[] {actual};
    }

    public DevicePointer(DeviceKey device, TalonFX[] actual) {
      this.device = device;
      this.actual = Arrays.copyOf(actual, actual.length);
    }

    public DeviceKey getDevice() {
      return device;
    }

    public TalonFX[] getActual() {
      return actual;
    }
  }

  /** Defines multiple logical-to-physical device mappings at once. */
  public void defineDevice(List<DevicePointer> pointers) {
    for (DevicePointer pointer : pointers) {
      devices.put(pointer.getDevice(), pointer.getActual());
    }
  }

  /** Defines one logical-to-physical device mapping. */
  public void defineDevice(DevicePointer pointer) {
    devices.put(pointer.getDevice(), pointer.getActual());
  }

  /**
   * Returns the physical motors for a logical device key.
   *
   * <p>Returns an empty array when the key is not defined to avoid null iteration failures.
   */
  public TalonFX[] getDevices(DeviceKey device) {
    TalonFX[] group = devices.get(device);
    if (group == null) {
      return new TalonFX[0];
    }
    return Arrays.copyOf(group, group.length);
  }
}
