package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public final class MotorOutputLimiter {
  public static final double DEFAULT_LIMIT_PERCENT = 1.0;

  private static double limitPercent = DEFAULT_LIMIT_PERCENT;

  private MotorOutputLimiter() {}

  public static void setLimitPercent(double percent) {
    limitPercent = MathUtil.clamp(percent, 0.0, 1.0);
    Logger.recordOutput("MotorOutputLimiter/Percent", limitPercent);
  }

  public static double getLimitPercent() {
    return limitPercent;
  }

  public static double scaleVoltage(double volts) {
    return volts * limitPercent;
  }

  public static double scaleCurrent(double amps) {
    return amps * limitPercent;
  }

  public static double scaleTorqueCurrent(double amps) {
    return scaleCurrent(amps);
  }

  public static double scaleFeedforward(double feedforward) {
    return feedforward * limitPercent;
  }
}
