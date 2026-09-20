// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;

/** Add your docs here. */
public class TrajectoryConfig {

  public double G = 9.80;
  public double DY = 1.2288; // 1.8288 - 0.6
  public boolean APEX_BEFORE_TARGET = true;

  public double min_hMax = 0;
  public double max_hMax = 4;
  public boolean useDistanceBasedHMax = false;
  public double midArcStartDistanceMeters;
  public double farArcStartDistanceMeters;
  public double nearHMax;
  public double midHMax;
  public double farHMax;

  public int POLYNOMIAL_DEGREE = 3;

  public double minHoodMechanismDeg = 0.0;
  public double maxHoodMechanismDeg = 25.0;
  public double minFlywheelRadPerSec = 0.0;
  public double maxFlywheelRadPerSec = FlywheelConstants.kShotMaxVelocity;

  // Optional calibrated mappings. Empty arrays keep the legacy mapping behavior.
  public double[] HOOD_MECHANISM_DEG_COEFFS = {};
  public double[] FLYWHEEL_RAD_PER_SEC_COEFFS = {};

  // Polynomial coefficients for aero correction. Features are generated in this order:
  // [1, dx, hmax, dx^2, dx*hmax, hmax^2, ...] up to POLYNOMIAL_DEGREE.
  public double[] DVX_COEFFS = {
    -0.08450618093881918,
    -0.009837347218053185,
    0.13674748925075123,
    -0.0030658470551465714,
    0.018822387782098767,
    -0.018255996814091676,
    -0.00011147978768855354,
    0.0010860553848757643,
    -0.003321669210630178,
    0.0036514134394532915
  };
  public double[] DVY_COEFFS = {
    -0.07230402829128711,
    -0.028479659883815272,
    0.088710520507098,
    -0.001715989222679634,
    0.003342169837324043,
    -0.018125628513105257,
    -0.00013078819069747838,
    0.0007362494446703192,
    -0.0015496733072326574,
    0.0024913578085115125
  };

  // with_spin
  // Replace the whole getHubConfig() method in TrajectoryConfig.java with this method.
  // Features: ['1', 'dx', 'hmax', 'dx^2', 'dx hmax', 'hmax^2', 'dx^3', 'dx^2 hmax', 'dx hmax^2',
  // 'hmax^3', 'dx^4', 'dx^3 hmax', 'dx^2 hmax^2', 'dx hmax^3', 'hmax^4', 'dx^5', 'dx^4 hmax', 'dx^3
  // hmax^2', 'dx^2 hmax^3', 'dx hmax^4', 'hmax^5']
  public static TrajectoryConfig getHubConfig() {
    TrajectoryConfig config = new TrajectoryConfig();
    config.G = 9.80;
    config.DY = 1.228800;
    config.APEX_BEFORE_TARGET = true;
    config.min_hMax = 2.1;
    config.max_hMax = 2.9;
    config.useDistanceBasedHMax = true;

    config.midArcStartDistanceMeters = 3.49;
    config.farArcStartDistanceMeters = 4.52;
    config.nearHMax = 2.2;
    config.midHMax = 2.5;
    config.farHMax = 2.8;
    config.POLYNOMIAL_DEGREE = 5;
    config.minHoodMechanismDeg = 0.0;
    config.maxHoodMechanismDeg = 25.0;
    config.minFlywheelRadPerSec = 0.0;
    config.maxFlywheelRadPerSec = FlywheelConstants.kShotMaxVelocity;

    config.DVX_COEFFS =
        new double[] {
          -0.0040623273563245299,
          0.006610240276212773,
          -0.00012169434970057983,
          0.0071949027088524546,
          0.0059155100066047121,
          -0.00011362274188515065,
          0.0012928043829946001,
          -0.0078439697862836902,
          0.0020373116917276933,
          0.00023653386680319661,
          -8.3692178001779153e-05,
          -0.00025381932352439598,
          0.002429664951425829,
          -0.001372681662948752,
          -3.5508126315072759e-06,
          2.1154750338861128e-06,
          8.99567528943122e-06,
          1.230387624925975e-05,
          -0.00023738503476424845,
          0.00018132688787174633,
          -8.05118337035641e-06
        };
    config.DVY_COEFFS =
        new double[] {
          0.022868390374062283,
          -0.00017773282616810992,
          0.0010228104059721572,
          0.0023515466809076783,
          -0.00054337178488718222,
          0.0031799253680210313,
          -0.00043972077347213665,
          0.0005061527111533111,
          -0.00080868191330003868,
          0.0048001584857442848,
          2.3748559040597187e-05,
          0.00011033309239100022,
          -0.0004138827126356835,
          0.00048945559755350166,
          -0.0013271889714706452,
          -5.5851072817686427e-07,
          -3.1260735924451729e-06,
          -6.2764927213384804e-06,
          5.1340724650034158e-05,
          -6.3945903774772196e-05,
          0.00011000439790932216
        };
    config.HOOD_MECHANISM_DEG_COEFFS =
        new double[] {45.234758835326907, -0.075688784008932961, -0.0061183009817124051};
    config.FLYWHEEL_RAD_PER_SEC_COEFFS = new double[] {85.330079512016141, 33.26311444724282};
    return config;
  }

  public static TrajectoryConfig getPassingConfig() {
    TrajectoryConfig config = getHubConfig();
    config.DY = -0.6;
    config.min_hMax = 1;
    config.max_hMax = 3;
    config.maxFlywheelRadPerSec = FlywheelConstants.kPassVelocity;
    config.useDistanceBasedHMax = false;
    return config;
  }
}
