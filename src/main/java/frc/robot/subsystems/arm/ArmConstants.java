package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int armLeftForwardMotorid = 14;
  public static final int armLeftBackMotorid = 15;
  public static final int armRightForwardMotorid = 16;
  public static final int armRightBackMotorid = 17;

  public static final double armRadius = 0.091;
  public static final double armGearRatio = 117.19;

  public static final double armPosition_Start = 1.07685; // TODO
  /*motion magic version arm */
  // public static final double kp = 100; //80;
  // public static final double Ki = 0.0;
  // public static final double Kd = 0.0;//0.0;
  // public static final double Kg = 0.5;
  // public static final double Ks = 0;
  // public static final double Kv = 0;
  // public static final double Ka = 0;
  // public static final double acceleration=1.5;
  // public static final double velocity=0.5;

  public static final double kp = 300.0; // 300;//450.0
  public static final double Ki = 0.0;
  public static final double Kd = 0.0; // 4.8*6;//55.0;50.0
  public static final double Kg = 0.0;
  public static final double Ks = 0.8; // 2.0; //7.9    public static final double Kv = 0.0;
  public static final double Kv = 0.0;
  public static final double Ka = 0.0;
  public static final double acceleration = 2;
  public static final double velocity = 1;
  public static final double torqueRatio = 100;

  public static final double kp_Climber = 80;
  public static final double Ki_Climber = 0.0;
  public static final double Kd_Climber = 0.8; // 0.0; s
  public static final double Kg_Climber = 0.9;
  public static final double Ks_Climber = 0;
  public static final double Kv_Climber = 0;
  public static final double Ka_Climber = 0;
  public static final double acceleration_Climber = 3;
  public static final double velocity_Climber = 1;

  public static final double armMinimumAngle = Math.toRadians(0); // radius
  public static final double armMaximumAngle = Math.toRadians(160); // radius
  public static final double armInitialAngle = 1.07685; // Math.toRadians(0);1.07685
  public static final double armToThetaCompensation = -20; // 0 radian is actually -20 theta degree

  public static final double armScoreL1Angle = Math.toRadians(155.0);
  public static final double armScoreL2Angle = Math.toRadians(45); // 47.5
  public static final double armScoreL3Angle = Math.toRadians(115);
  public static final double armScoreL4Angle = Math.toRadians(105);
  public static final double armStationIntakeAngle = Math.toRadians(51);
  public static final double armRestAngle = Math.toRadians(45);

  public static final double armTestAngle = Math.toRadians(110);

  public static final double armClimbDownAngle = Math.toRadians(20);
  public static final double armClimbUpAngle = Math.toRadians(20);

  public static final double armTolerance = Math.toRadians(7);

  public static final double motorKT =
      9.37 / 483; // 7.09/366;//7.09 is stall Torque , 366 is stall current

  public static final double testUpVolts = 5.0;
  public static final double testDownVolts = -5.0;

  // Sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0);
  public static final double jkMetersSquared = 1.06328;
  public static final double armLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
}
