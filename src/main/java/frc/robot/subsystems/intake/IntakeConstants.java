package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int id = 13;
  public static final int intakePhotoSensorChannel = 0;

  public static final double intakeGearRatio = 3.333; // Gear Ratios
  public static final double wheelRadius = 0.0381;

  public static final double Kp = 0.7;
  public static final double Ki = 0.0;
  public static final double Kd = 0.03;
  public static final double Ks = 0.14185;
  public static final double Kv = 0.13;
  public static final double Ka = 0.30801;

  public static final double intakeVolts = -2;
  public static final double scoreForwardVolts = 1.5; // L1, L2
  public static final double scoreForwardL4Volts = -4; // L1, L2
  public static final double reverse4L4Volts =
      1.5; // reverse until sensor false, for level 4 shooting
  public static final double ejectVolts = -10.0;

  public static final double TorqueCurrentAmps = 30.0;
  public static final double VelocityMax = 4.0;
  public static final double AccelerationMax = 2.0;
  public static final double Velocity1 = -10.0; // IntakeCaptureVel
  public static final double Acceleration1 = 4.0; // //IntakeCaptureAcc

  public static final double scoringLastTime = 1.2; // //IntakeCaptureAcc
  public static final double capturingLastTime = 1; // //IntakeCaptureAcc
}
