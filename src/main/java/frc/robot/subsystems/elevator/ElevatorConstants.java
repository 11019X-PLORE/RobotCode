package frc.robot.subsystems.elevator;

import frc.robot.subsystems.arm.ArmConstants;

public class ElevatorConstants {
  public static final int elevatorLeftId = 18;
  public static final int elevatorRightId = 19;

  public static final double elevatorRadius = 0.04064;
  public static final double elevatorGearRatio = 4.27;

  public static final double kp = 150.0; // 100
  public static final double Ki = 0.0;
  public static final double Kd = 11.0; // 0.0;
  public static final double Kg = 0.0;
  public static final double Ks = 14.23;
  public static final double Kv = 0;
  public static final double Ka = 0;

  public static final double forceAjust = 1.25;

  public static final double Kt = 0.1;

  public static final double acceleration = 2; // 10
  public static final double velocity = 1; // 4

  public static final double elevatorMinimumPosition = 0.0; // meters
  public static final double elevatorMaximumPosition = 0.875; // meters
  public static final double elevatorInitialPosition = 0.0;
  public static final double elevatorMaximumQuantityCenterPositin = 0.354; // meters
  public static final double elevatorMinimumQuantityCenterPositin = 0.311; // meters

  public static final double elevatorScoreL1Position = 0.0;
  public static final double elevatorScoreL2Position = 0.2;
  public static final double elevatorScoreL3Position = 0.4;
  public static final double elevatorScoreL4Position = 1.1; //
  public static final double elevatorStationIntakePosition = 0.01;
  public static final double elevatorRestPosition = 0.0;
  public static final double elevatorTestPosition = 0.6;

  public static final double torqueL4 =
      elevatorScoreL4Position * Kt * Math.sin(ArmConstants.armScoreL4Angle);

  public static final double elevatorClimbDownPosition = 0.2;
  public static final double elevatorClimbUpPosition = 0.2;

  public static final double elevatorTolerance = 0.05;

  public static final double testUpVolts = 5.0;
  public static final double testDownVolts = -5.0;

  // Sim
  public static final double jkMetersSquared = 1.06328;
  public static final double elevatorLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
  public static final double drumRadiusMeters = 0.494;
}
