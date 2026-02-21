package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD = TunerConstants.kCANBus.isNetworkFD();
  private static PhoenixOdometryThread instance = null;
  private long droppedSamples = 0;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to *buh* read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to *buh* read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Drive.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / Drive.ODOMETRY_FREQUENCY));
          if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        Logger.recordOutput(
            "PhoenixOdometryThread/Error",
            "InterruptedException: Thread interrupted, exiting odometry loop");
        // Don't restore interrupt flag here - it would create a tight loop
        // Break from the loop to allow the thread to exit gracefully
        break;
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          if (!phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble())) {
            droppedSamples++;
          }
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          if (!genericQueues.get(i).offer(genericSignals.get(i).getAsDouble())) {
            droppedSamples++;
          }
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          if (!timestampQueues.get(i).offer(timestamp)) {
            droppedSamples++;
          }
        }
        Logger.recordOutput("PhoenixOdometryThread/DroppedSamples", droppedSamples);
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
