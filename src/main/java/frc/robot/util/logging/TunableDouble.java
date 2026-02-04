package frc.robot.util.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class TunableDouble implements DoubleSupplier {
  private final String key;
  private final BooleanSupplier shouldPublish;
  private final DoubleConsumer onChange;
  private double value;
  // private LoggedNetworkNumber networkNumber = null;

  public TunableDouble(
      String key, double defaultValue, BooleanSupplier shouldPublish, DoubleConsumer onChange) {
    this.key = key;
    this.shouldPublish = shouldPublish;
    this.onChange = onChange;
    value = defaultValue;
    SmartDashboard.putNumber(key, value);
    LogUtil.getInstance().registerUpdateMethod(this::update);
  }

  public TunableDouble(String key, double defaultValue, BooleanSupplier shouldPublish) {
    this(key, defaultValue, shouldPublish, (value) -> {});
  }

  public void update() {
    // if (shouldPublish.getAsBoolean()) {
    //   if (networkNumber == null) {
    //     networkNumber = new LoggedNetworkNumber(key, value);
    //   }
      if (value != SmartDashboard.getNumber(key, value)) {
        value = SmartDashboard.getNumber(key, value);
        onChange.accept(value);
      }
    // } else {
    //   if (networkNumber != null) {
    //     networkNumber = null;
    //     NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(value).unpublish();
    //   }
    // }
  }

  @Override
  public double getAsDouble() {
    return value;
  }
}
