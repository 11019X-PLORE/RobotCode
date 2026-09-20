package frc.robot.util.Geoffrey;

import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;

public class TrajectoryCalculator {

  /**
   * Compute launch velocities (vx, vy) for a given horizontal distance and desired apex height.
   *
   * @param dx horizontal distance to target (m)
   * @param hmax desired apex height (m), must be greater than DY
   * @return double[]{vx, vy} in m/s
   */
  public static double[] solve(double dx, double hmax, TrajectoryConfig config) {
    // 1. Vacuum baseline from kinematics
    double vyVac = Math.sqrt(2.0 * config.G * hmax);
    double vxVac;

    if (Math.abs(config.DY) < 1e-10) {
      vxVac = config.G * dx / (2.0 * vyVac);
    } else {
      double disc = vyVac * vyVac - 2.0 * config.G * config.DY;
      if (disc < 0) {
        return new double[] {0, 0}; // impossible trajectory
      }
      double sqrtDisc = Math.sqrt(disc);
      if (config.APEX_BEFORE_TARGET) {
        vxVac = dx * (vyVac - sqrtDisc) / (2.0 * config.DY);
      } else {
        vxVac = dx * (vyVac + sqrtDisc) / (2.0 * config.DY);
      }
    }

    // 2. Polynomial aero correction.
    double[] features = polynomialFeatures(dx, hmax, config.POLYNOMIAL_DEGREE);

    double dvx = dot(config.DVX_COEFFS, features);
    double dvy = dot(config.DVY_COEFFS, features);

    // 3. Final velocity = vacuum + aero correction
    return new double[] {vxVac + dvx, vyVac + dvy};
    // return new double[] {vxVac, vyVac};
  }

  private static double dot(double[] a, double[] b) {
    double sum = 0;
    for (int i = 0; i < a.length; i++) {
      sum += a[i] * b[i];
    }
    return sum;
  }

  private static double[] polynomialFeatures(double dx, double hmax, int degree) {
    int featureCount = (degree + 1) * (degree + 2) / 2;
    double[] features = new double[featureCount];
    int idx = 0;

    for (int totalPower = 0; totalPower <= degree; totalPower++) {
      for (int dxPower = totalPower; dxPower >= 0; dxPower--) {
        int hPower = totalPower - dxPower;
        features[idx++] = Math.pow(dx, dxPower) * Math.pow(hmax, hPower);
      }
    }

    return features;
  }

  private static double[] polynomialFeatureDerivativeDx(double dx, double hmax, int degree) {
    int featureCount = (degree + 1) * (degree + 2) / 2;
    double[] features = new double[featureCount];
    int idx = 0;

    for (int totalPower = 0; totalPower <= degree; totalPower++) {
      for (int dxPower = totalPower; dxPower >= 0; dxPower--) {
        int hPower = totalPower - dxPower;
        if (dxPower == 0) {
          features[idx++] = 0.0;
        } else {
          features[idx++] = dxPower * Math.pow(dx, dxPower - 1) * Math.pow(hmax, hPower);
        }
      }
    }

    return features;
  }

  private static double evalPolynomial(double[] coeffs, double x) {
    double y = 0.0;
    for (int i = coeffs.length - 1; i >= 0; i--) {
      y = y * x + coeffs[i];
    }
    return y;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public static double[] solveDerivative(double dx, double hmax, TrajectoryConfig config) {
    // --- 1. Derivative of Vacuum Components ---
    // vyVac = sqrt(2 * G * hmax). Since hmax is constant w.r.t dx, d(vyVac)/dx = 0
    double d_vyVac_dx = 0;

    double d_vxVac_dx;
    double vyVac = Math.sqrt(2.0 * config.G * hmax);

    if (Math.abs(config.DY) < 1e-10) {
      // vxVac = (G / 2*vyVac) * dx
      d_vxVac_dx = config.G / (2.0 * vyVac);
    } else {
      double disc = vyVac * vyVac - 2.0 * config.G * config.DY;
      if (disc < 0) return new double[] {0, 0};

      double sqrtDisc = Math.sqrt(disc);
      // vxVac is essentially (Constant * dx), so the derivative is just the constant
      if (config.APEX_BEFORE_TARGET) {
        d_vxVac_dx = (vyVac - sqrtDisc) / (2.0 * config.DY);
      } else {
        d_vxVac_dx = (vyVac + sqrtDisc) / (2.0 * config.DY);
      }
    }

    // --- 2. Derivative of Aero Correction ---
    double[] dFeatures = polynomialFeatureDerivativeDx(dx, hmax, config.POLYNOMIAL_DEGREE);

    double d_dvx_dx = dot(config.DVX_COEFFS, dFeatures);
    double d_dvy_dx = dot(config.DVY_COEFFS, dFeatures);

    // --- 3. Final Summation ---
    // d(Total)/dx = d(Vacuum)/dx + d(Aero)/dx
    return new double[] {d_vxVac_dx + d_dvx_dx, d_vyVac_dx + d_dvy_dx};
  }

  /**
   * Compute the required flywheel surface speed for a given total ball speed.
   *
   * @param v total ball speed (m/s)
   * @return flywheel surface speed (m/s)
   */
  public static double getFlywheelSetpoint(double v) {
    // if (v > 7.3) {
    //   return 4.74446 * Math.pow(1.17377, v);
    // }
    // return 15;
    // return 0.85 * 4.40309 * Math.pow(1.17819, v);
    // return 4.62907 * Math.pow(1.18664, v); // placeholder for testing
    // return 3.44691 * v - 9.06894;

    return 1.1 * 4.44865 * Math.pow(1.14391, v);
  }

  public static double getFlywheelSetpointRadPerSec(double v, TrajectoryConfig config) {
    double velocityRadPerSec;
    if (config.FLYWHEEL_RAD_PER_SEC_COEFFS.length == 0) {
      velocityRadPerSec = getFlywheelSetpoint(v) / FlywheelConstants.kFlywheelRadius;
    } else {
      velocityRadPerSec = evalPolynomial(config.FLYWHEEL_RAD_PER_SEC_COEFFS, v);
    }

    return clamp(velocityRadPerSec, config.minFlywheelRadPerSec, config.maxFlywheelRadPerSec);
  }

  public static double getHoodSetpoint(double a) {
    return Math.toRadians((0.746204 * Math.toDegrees(a)) + 14.48929);
    // return Math.toRadians((0.901026 * Math.toDegrees(a)) + 5.23916);

    // return Math.toRadians((0.0774547 * Math.pow(a, 1.5867)));
    // return a;
  }

  public static double getHoodSetpoint(double launchAngleRads, TrajectoryConfig config) {
    if (config.HOOD_MECHANISM_DEG_COEFFS.length == 0) {
      return getHoodSetpoint(launchAngleRads);
    }

    double hoodMechanismDeg =
        clamp(
            evalPolynomial(config.HOOD_MECHANISM_DEG_COEFFS, Math.toDegrees(launchAngleRads)),
            config.minHoodMechanismDeg,
            config.maxHoodMechanismDeg);

    // Hood.runPositionFOCLogic converts targetAngle with (PI / 2) - targetAngle internally.
    // The calibrated mapping is in mechanism degrees, so send the complementary launch angle.
    return Math.toRadians(90.0 - hoodMechanismDeg);
  }

  public static double getFlywheelAcceleration(double v, double a) {
    double dx = 1e-6; // small delta for numerical differentiation
    double dv = getFlywheelSetpoint(v + dx) - getFlywheelSetpoint(v);
    return (dv / dx) * a;
  }

  public static double getFlywheelAccelerationRadPerSec2(
      double v, double a, TrajectoryConfig config) {
    double dx = 1e-6;
    double dv =
        getFlywheelSetpointRadPerSec(v + dx, config) - getFlywheelSetpointRadPerSec(v, config);
    return (dv / dx) * a;
  }
}
