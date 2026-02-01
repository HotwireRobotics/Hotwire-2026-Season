package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModularSubsystem extends SubsystemBase {
  private final HashMap<Object, Object> devices = new HashMap<Object, Object>();
  private List<Object> active = new ArrayList<Object>();

  public void specifyActiveDevice(Object device) {
    active.add(device);
  }
  public void specifyInactiveDevice(Object device) {
    active.remove(device);
  }
  public boolean isActiveDevice(Object device) {
    return active.contains(device);
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
