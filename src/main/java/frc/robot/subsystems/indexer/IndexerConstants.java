package frc.robot.subsystems.indexer;

import frc.robot.Ports;

public class IndexerConstants {
  // === Indexer Motor ===
  public static final int kIndexerId = Ports.kIndexerRollers;
  public static boolean kIndexInverted = true;

  public static final double kIndexerRadius = 0.05; // TODO
  public static final double kIndexerGearRatio = 1.5;
  public static final double kIndexerStatorCurrentLimit = 50;

  public static final double kP = 60.0; // 10.0
  public static final double kI = 0.0;
  public static final double kD = 0.0; // 0.0;
  public static final double kG = 0.0; // 重力补偿
  public static final double kS = 0.08; // 静态前馈
  public static final double kV = 0.16; // 轨迹规划得到的目标速度
  public static final double kA = 0.0; // 轨迹规划得到的目标加速度

  public static final double kVelocityTolerance = 5.0;

  public static final double kIntakeVelocity = 400.0;
  public static final double kOuttakeVelocity = -400.0;
  public static final double kActiveVelocity = 100.0;
  public static final double kShootVelocity = 400.0;

  // === Rotator Motors (merged from RotatorConstants) ===

  public static final int kRotatorId = Ports.kRotatorRollers;
  public static boolean kRotatorInverted = false;
  public static final double kRotatorGearRatio = 10.0;

  public static double kRotatorMinSwitchPeriod = 0.2;

  public static final double kRotator_kP = 45.0;
  public static final double kRotator_kI = 0.0;
  public static final double kRotator_kD = 0.0;
  public static final double kRotator_kG = 0.0;
  public static final double kRotator_kS = 0.0;
  public static final double kRotator_kV = 0.0;
  public static final double kRotator_kA = 0.0;

  public static final double kRotatorVelocityTolerance = 5.0;

  public static final double kRotatorIntakeVelocity = 50.0;
  public static final double kRotatorOuttakeVelocity = 30.0;
  public static final double kRotatorShootVelocity = 50.0;

  public static final double kRotatorStallVelocityThreshold = 5.0;
  public static final double kRotatorStallTorqueCurrentThreshold = 80.0;
  public static final double kRotatorStallDebounceSeconds = 0.25;
  public static final double kRotatorOuttakeReverseSeconds = 5.0;
  public static final double kRotatorOuttakeForwardSeconds = 1.0;
  public static final double kRotatorRecoverySeconds = 2.5;
  public static final int kRotatorRecoveryMaxAttempts = 5;
  public static final double kRotatorMaxAccelerationRadPerSecSq = 60.0;

  public static final double feedingBPS = 6.0; // balls per second
}
