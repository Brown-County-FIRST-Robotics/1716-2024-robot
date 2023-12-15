package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;

public class DualRateLimiter {
  private final double accelRate;
  private final double deccelRate;
  private double prevVal;
  private double prevTime;

  public DualRateLimiter(double accelRate, double deccelRate, double prevVal) {
    this.accelRate = accelRate;
    this.deccelRate = deccelRate;
    this.prevVal = prevVal;
    this.prevTime = MathSharedStore.getTimestamp();
  }

  private static double clamp(double v, double mn, double mx) {
    return Math.min(mx, Math.max(v, mn));
  }

  public DualRateLimiter(double accelRate, double deccelRate) {
    this(accelRate, deccelRate, 0);
  }

  public double calculate(double val) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        clamp(
                (val - prevVal) * Math.signum(prevVal == 0 ? Double.MIN_VALUE : prevVal),
                -deccelRate * elapsedTime,
                accelRate * elapsedTime)
            * Math.signum(prevVal == 0 ? Double.MIN_VALUE : prevVal);
    prevTime = currentTime;
    return prevVal;
  }

  public void reset(double val) {
    prevTime = MathSharedStore.getTimestamp();
    prevVal = val;
  }
}
