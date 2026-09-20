package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FullSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

public class Vision extends FullSubsystem {
  private static final Pose3d kPose3dIdentity = Pose3d.kZero;
  private static final Pose3d[] kEmptyPose3dArray = new Pose3d[0];

  private final Drive drive;
  // private final Drive drive;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final TimeInterpolatableBuffer<Pose3d>[] robot2cameraPoseBuffer;

  // Pre-allocated lists reused every cycle (cleared at start of periodicAfterScheduler)
  private final List<Pose3d> allTagPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPoses = new ArrayList<>();
  private final List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private final List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  private final List<Pose3d>[] perCamTagPoses;
  private final List<Pose3d>[] perCamRobotPoses;
  private final List<Pose3d>[] perCamRobotPosesAccepted;
  private final List<Pose3d>[] perCamRobotPosesRejected;

  // Pre-computed log key strings
  private final String[] camTagPosesKeys;
  private final String[] camRobotPosesKeys;
  private final String[] camRobotPosesAcceptedKeys;
  private final String[] camRobotPosesRejectedKeys;

  // Reusable std dev matrix
  private final Matrix<N3, N1> stdDevMatrix = VecBuilder.fill(0, 0, 0);

  private boolean visionReferenceEstablished = false;

  public Vision(Drive drive, VisionIO... io) {
    // this.drive = drive;
    this.drive = drive;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    robot2cameraPoseBuffer =
        new TimeInterpolatableBuffer[io.length]; // Separate buffer for each camera
    for (int i = 0; i < io.length; i++) {
      robot2cameraPoseBuffer[i] = TimeInterpolatableBuffer.createBuffer(2.0);
    }

    // Pre-allocate per-camera lists and log key strings
    int numCameras = io.length;
    perCamTagPoses = new List[numCameras];
    perCamRobotPoses = new List[numCameras];
    perCamRobotPosesAccepted = new List[numCameras];
    perCamRobotPosesRejected = new List[numCameras];
    camTagPosesKeys = new String[numCameras];
    camRobotPosesKeys = new String[numCameras];
    camRobotPosesAcceptedKeys = new String[numCameras];
    camRobotPosesRejectedKeys = new String[numCameras];
    for (int i = 0; i < numCameras; i++) {
      perCamTagPoses[i] = new ArrayList<>();
      perCamRobotPoses[i] = new ArrayList<>();
      perCamRobotPosesAccepted[i] = new ArrayList<>();
      perCamRobotPosesRejected[i] = new ArrayList<>();
      String prefix = "Vision/Camera" + i;
      camTagPosesKeys[i] = prefix + "/TagPoses";
      camRobotPosesKeys[i] = prefix + "/RobotPoses";
      camRobotPosesAcceptedKeys[i] = prefix + "/RobotPosesAccepted";
      camRobotPosesRejectedKeys[i] = prefix + "/RobotPosesRejected";
      Logger.recordOutput(prefix + "/RejectedForOtherCameraMultiTag", false);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void updateInputsPeriodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      // Compute the camera pose in the robot (base) frame:
      //   T_robot_to_camera = T_world_to_robot^-1 ⊕ T_world_to_camera
      // where T_world_to_camera = mountingEnd.globalTip ⊕ mountingOffset
      Transform3d cameraGlobal =
          io[i].getBaseJoint().getGlobalPose().plus(io[i].getMountingOffset());
      Transform3d robotGlobal = drive.getGlobalPose();
      // robot^-1 ⊕ camera  →  camera expressed in robot frame
      Pose3d cameraInRobot = kPose3dIdentity.transformBy(robotGlobal.inverse().plus(cameraGlobal));
      this.robot2cameraPoseBuffer[i].addSample(Timer.getFPGATimestamp(), cameraInRobot);
    }

    // Clear reusable summary lists
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Clear reusable per-camera lists
      List<Pose3d> tagPoses = perCamTagPoses[cameraIndex];
      List<Pose3d> robotPoses = perCamRobotPoses[cameraIndex];
      List<Pose3d> robotPosesAccepted = perCamRobotPosesAccepted[cameraIndex];
      List<Pose3d> robotPosesRejected = perCamRobotPosesRejected[cameraIndex];
      boolean rejectedForOtherCameraMultiTagThisCycle = false;
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      SimpleMatrix baseV = io[cameraIndex].getBaseJoint().getGlobalVelocity();

      double baseSpeed = Math.hypot(baseV.get(0, 0), baseV.get(1, 0));
      double baseAngularSpeed = Math.abs(baseV.get(5, 0));

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        var sampleRobotPoseOptional = drive.getPose(observation.timestamp());
        if (sampleRobotPoseOptional.isEmpty()) {
          continue;
        }
        var sampleRobotPose = sampleRobotPoseOptional.get();

        Pose3d visionPose3d;
        if (observation.type() == VisionIO.PoseObservationType.MEGATAG_1
            || observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
          // Limelight botpose_wpiblue and botpose_orb_wpiblue are already robot poses in field
          // coordinates.
          visionPose3d = observation.pose();
        } else if (observation.type() == VisionIO.PoseObservationType.PHOTONVISION) {
          var robot2cameraPoseOptional =
              robot2cameraPoseBuffer[cameraIndex].getSample(observation.timestamp());
          if (robot2cameraPoseOptional.isEmpty()) {
            continue;
          }
          visionPose3d =
              observation
                  .pose()
                  .transformBy(
                      new Transform3d(kPose3dIdentity, robot2cameraPoseOptional.get()).inverse());
        } else {
          continue; // Skip unsupported observation types
        }

        boolean isMegaTag2 = observation.type() == VisionIO.PoseObservationType.MEGATAG_2;
        boolean isLimelightObservation =
            observation.type() == VisionIO.PoseObservationType.MEGATAG_1 || isMegaTag2;

        // Limelight heading is intentionally not fused, so a heading mismatch must not reject an
        // otherwise valid translation measurement.
        boolean rejectForYawMismatch =
            !isLimelightObservation
                && Math.abs(
                        Rotation2d.fromRadians(visionPose3d.getRotation().getZ())
                            .minus(sampleRobotPose.getRotation())
                            .getRadians())
                    > maxYawDifference;

        boolean rejectBasicPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                // On the official field, a solved robot pose must remain close to floor height.
                || Math.abs(visionPose3d.getZ()) > maxZError
                || rejectForYawMismatch
                || (isMegaTag2
                    && Math.abs(drive.getGyroRateDegPerSec()) > maxMegaTag2GyroRateDegPerSec)
                // Must be within the field boundaries
                || visionPose3d.getX() < 0.0
                || visionPose3d.getX() > aprilTagLayout.getFieldLength()
                || visionPose3d.getY() < 0.0
                || visionPose3d.getY() > aprilTagLayout.getFieldWidth();

        Pose2d visionPose2d = visionPose3d.toPose2d();
        double translationInnovationMeters =
            visionPose2d.getTranslation().getDistance(sampleRobotPose.getTranslation());
        double translationInnovationThresholdMeters =
            maxVisionTranslationInnovationMeters
                + baseSpeed * visionInnovationVelocityAllowanceSeconds;

        boolean rejectForTranslationInnovation =
            shouldRejectForTranslationInnovation(
                rejectBasicPose,
                visionReferenceEstablished,
                isLimelightObservation,
                translationInnovationMeters,
                translationInnovationThresholdMeters);

        boolean rejectForOtherCameraMultiTag =
            !rejectBasicPose
                && !rejectForTranslationInnovation
                && shouldRejectForOtherCameraMultiTag(cameraIndex, observation);
        rejectedForOtherCameraMultiTagThisCycle |= rejectForOtherCameraMultiTag;

        boolean rejectPose =
            rejectBasicPose || rejectForTranslationInnovation || rejectForOtherCameraMultiTag;

        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/TranslationInnovationMeters",
            translationInnovationMeters);
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/TranslationInnovationThresholdMeters",
            translationInnovationThresholdMeters);
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/RejectedForTranslationInnovation",
            rejectForTranslationInnovation);
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/TranslationInnovationBypassedForLimelight",
            !rejectBasicPose
                && isLimelightObservation
                && translationInnovationMeters > translationInnovationThresholdMeters);
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RejectedBasicPose", rejectBasicPose);
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RejectedPose", rejectPose);
        Logger.recordOutput(
            "Vision/Initialization/ReferenceEstablished", visionReferenceEstablished);

        // Add pose to log
        robotPoses.add(visionPose3d);
        if (rejectPose) {
          robotPosesRejected.add(visionPose3d);
        } else {
          robotPosesAccepted.add(visionPose3d);
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        visionReferenceEstablished = true;

        // Limelight 返回的是标签在画面中的平均面积百分比；面积越小，测量标准差越大。
        // 例如 1.0 表示可见标签平均占画面的 1%。
        double stdDevFactor =
            Math.max(
                1.0,
                targetAreaReferencePercent
                    / Math.max(observation.averageTagAreaPercent(), minTargetAreaPercent));

        double linearStdDev =
            linearStdDevBaseline * stdDevFactor + baseSpeed * VisionConstants.latencyStdDev;
        double angularStdDev =
            angularStdDevBaseline * stdDevFactor + baseAngularSpeed * VisionConstants.latencyStdDev;

        if (isMegaTag2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          linearStdDev = Math.max(linearStdDev, minMegaTag2LinearStdDevMeters);
          angularStdDev *= angularStdDevMegatag2Factor;
        } else if (observation.type() == VisionIO.PoseObservationType.MEGATAG_1) {
          angularStdDev = Double.POSITIVE_INFINITY;
        }
        linearStdDev = Math.max(linearStdDev, minLinearStdDevMeters);

        // 应用每个摄像头的独立调整系数
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // The timestamp is the image capture time. PoseEstimator rewinds to that time before
        // applying the correction, preserving latency compensation for the direct robot pose.
        stdDevMatrix.set(0, 0, linearStdDev);
        stdDevMatrix.set(1, 0, linearStdDev);
        stdDevMatrix.set(2, 0, angularStdDev);
        // Send vision observation
        // stdDevMatrix.set(0, 0, 0.1);
        // stdDevMatrix.set(1, 0, 0.1);
        // stdDevMatrix.set(2, 0, 1e9);
        drive.addVisionMeasurement(visionPose2d, observation.timestamp(), stdDevMatrix);
      }

      // Log camera metadata
      Logger.recordOutput(camTagPosesKeys[cameraIndex], tagPoses.toArray(kEmptyPose3dArray));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RejectedForOtherCameraMultiTag",
          rejectedForOtherCameraMultiTagThisCycle);
      Logger.recordOutput(camRobotPosesKeys[cameraIndex], robotPoses.toArray(kEmptyPose3dArray));
      Logger.recordOutput(
          camRobotPosesAcceptedKeys[cameraIndex], robotPosesAccepted.toArray(kEmptyPose3dArray));
      Logger.recordOutput(
          camRobotPosesRejectedKeys[cameraIndex], robotPosesRejected.toArray(kEmptyPose3dArray));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(kEmptyPose3dArray));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(kEmptyPose3dArray));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(kEmptyPose3dArray));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(kEmptyPose3dArray));
  }

  static boolean shouldRejectForTranslationInnovation(
      boolean rejectBasicPose,
      boolean visionReferenceEstablished,
      boolean isLimelightObservation,
      double translationInnovationMeters,
      double translationInnovationThresholdMeters) {
    return !rejectBasicPose
        && !isLimelightObservation
        && visionReferenceEstablished
        && translationInnovationMeters > translationInnovationThresholdMeters;
  }

  private boolean shouldRejectForOtherCameraMultiTag(
      int cameraIndex, VisionIO.PoseObservation observation) {
    if (!isLimelightObservation(observation) || observation.tagCount() != 1) {
      return false;
    }

    for (int otherCameraIndex = 0; otherCameraIndex < inputs.length; otherCameraIndex++) {
      if (otherCameraIndex == cameraIndex) {
        continue;
      }

      for (var otherObservation : inputs[otherCameraIndex].poseObservations) {
        if (isLimelightObservation(otherObservation)
            && otherObservation.tagCount() >= 2
            && Math.abs(otherObservation.timestamp() - observation.timestamp())
                <= crossCameraMultiTagTimestampToleranceSeconds) {
          return true;
        }
      }
    }

    return false;
  }

  private static boolean isLimelightObservation(VisionIO.PoseObservation observation) {
    return observation.type() == VisionIO.PoseObservationType.MEGATAG_1
        || observation.type() == VisionIO.PoseObservationType.MEGATAG_2;
  }
}
