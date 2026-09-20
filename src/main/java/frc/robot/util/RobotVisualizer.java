package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private final Drive drive;
  private final Extension extension;
  private final Turret turret;
  private final Hood hood;

  public RobotVisualizer(Drive drive, Extension extension, Turret turret, Hood hood) {
    this.drive = drive;
    this.extension = extension;
    this.turret = turret;
    this.hood = hood;
  }

  public void periodic() {
    Logger.recordOutput("RobotVisualizer/RobotPose", new Pose3d(drive.getPose()));
    Logger.recordOutput(
        "RobotVisualizer/ComponentPoses",
        RobotVisualizerGeometry.createComponentPoses(
            extension.getMeasuredAngleRad(), turret.getPosition(), hood.getMeasuredAngleRad()));
  }
}
