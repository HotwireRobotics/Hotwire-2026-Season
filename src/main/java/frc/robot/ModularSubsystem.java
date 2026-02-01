package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ModularSubsystem extends SubsystemBase {
  private final HashMap<Object, Object> devices = new HashMap<Object, Object>();
  private List<Object> active = new ArrayList<Object>();

  public void specifyActiveDevice(Object device) {
    for (TalonFX d : getDevices(device)) {
      active.add(d);
    }
  }

  public void specifyInactiveDevice(Object device) {
    for (TalonFX d : getDevices(device)) {
      active.remove(d);
    }
  }

  public boolean isActiveDevice(Object device) {
    boolean isActive = false;
    for (TalonFX d : getDevices(device)) {
      isActive = isActive || active.contains(d);
    }
    return isActive;
  }

  public void defineDevice(Object device, TalonFX actual) {
    devices.put(device, actual);
  }

  public void defineDevice(Object device, TalonFX[] actual) {
    devices.put(device, actual);
  }

  public TalonFX[] getDevices(Object device) {
    Object group = devices.get(device);
    if (group instanceof TalonFX[]) {
      return ((TalonFX[]) group);
    } else {
      TalonFX[] items = {(TalonFX) group};
      return items;
    }
  }
}
