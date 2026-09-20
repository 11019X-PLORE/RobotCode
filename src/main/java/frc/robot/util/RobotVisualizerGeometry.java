package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shooter.turret.TurretConstants;

public final class RobotVisualizerGeometry {
  public static final int INTAKE_INDEX = 0;
  public static final int TURRET_INDEX = 1;
  public static final int COMPONENT_COUNT = 2;

  public static final double TURRET_Z_METERS = 0.407;

  private static final Translation3d INTAKE_PIVOT = new Translation3d(0.14075, 0.0, 0.24);

  private RobotVisualizerGeometry() {}

  public static Pose3d[] createComponentPoses(
      double intakeAngleRad, double turretAngleRad, double hoodAngleRad) {
    Pose3d intakePose = new Pose3d(INTAKE_PIVOT, new Rotation3d(0.0, -intakeAngleRad, 0.0));
    Pose3d turretPose = createTurretPose(turretAngleRad);

    Pose3d[] poses = new Pose3d[COMPONENT_COUNT];
    poses[INTAKE_INDEX] = intakePose;
    poses[TURRET_INDEX] = turretPose;
    return poses;
  }

  private static Pose3d createTurretPose(double turretAngleRad) {
    return new Pose3d(
        TurretConstants.kTurretonRobotoffset.getX(),
        TurretConstants.kTurretonRobotoffset.getY(),
        TURRET_Z_METERS,
        new Rotation3d(0.0, 0.0, turretAngleRad));
  }
}
