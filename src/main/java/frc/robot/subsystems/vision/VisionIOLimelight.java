package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.validFiducialIds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;
import frc.robot.util.Geoffrey.PhysicalJoint;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleSupplier yawRateSupplierDegPerSec;

  private final DoubleSubscriber heartbeatSubscriber;
  private final DoubleSubscriber tvSubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final PhysicalJoint baseJoint;
  private final Transform3d mountingOffset;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Current robot field heading, required by MegaTag 2.
   * @param yawRateSupplierDegPerSec Current robot yaw rate in degrees per second.
   * @param baseJoint The fixed robot joint the camera is mounted to.
   * @param mountingOffset The transform from the base joint to the camera.
   */
  public VisionIOLimelight(
      String name,
      Supplier<Rotation2d> rotationSupplier,
      DoubleSupplier yawRateSupplierDegPerSec,
      PhysicalJoint baseJoint,
      Transform3d mountingOffset) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.rotationSupplier = rotationSupplier;
    this.yawRateSupplierDegPerSec = yawRateSupplierDegPerSec;
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    this.baseJoint = baseJoint;

    this.mountingOffset = mountingOffset;

    // Clear any old test-only filter by explicitly enabling every tag in the field layout.
    LimelightHelpers.SetFiducialIDFiltersOverride(name, validFiducialIds);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - heartbeatSubscriber.getLastChange()) / 1000) < 250;
    inputs.hasTarget = tvSubscriber.get() >= 1.0;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // MegaTag 2 uses the gyro heading to resolve a single-tag observation. The returned heading is
    // not fused back into the estimator, avoiding feedback of the same gyro measurement.
    LimelightHelpers.SetRobotOrientation_NoFlush(
        name,
        rotationSupplier.get().getDegrees(),
        yawRateSupplierDegPerSec.getAsDouble(),
        0.0,
        0.0,
        0.0,
        0.0);
    NetworkTableInstance.getDefault().flush();

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();

    List<PoseObservation> poseObservations = new LinkedList<>();
    List<PoseObservation> megatag1Observations = new LinkedList<>();
    List<PoseObservation> megatag2Observations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      // Limelight pose estimates contain 11 summary values followed by seven values
      // for every detected fiducial.
      if (rawSample.value.length < 11) continue;
      int tagCount = (int) rawSample.value[7];
      if (tagCount <= 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      megatag1Observations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              tagCount,

              // Average tag distance
              rawSample.value[9],
              // Average fiducial area as a percentage of the image.
              rawSample.value[10],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length < 11) continue;
      int tagCount = (int) rawSample.value[7];
      if (tagCount <= 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      megatag2Observations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and total latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // MegaTag 2 robot pose in the blue-origin field coordinate system
              parsePose(rawSample.value),

              // MegaTag 2 uses the supplied gyro heading to resolve ambiguity
              0.0,

              // Tag count
              tagCount,

              // Average tag distance
              rawSample.value[9],

              // Average fiducial area as a percentage of the image
              rawSample.value[10],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // MegaTag 1 and MegaTag 2 are calculated from the same camera frame, so fusing both would
    // double-count one measurement. Prefer MegaTag 2 and retain MegaTag 1 only as a fallback.
    poseObservations.addAll(
        megatag2Observations.isEmpty() ? megatag1Observations : megatag2Observations);

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    // if (inputs.tagIds.length != 0) {
    //   inputs.tagIds[0] = primaryID;
    //   int i = 1;
    //   for (int id : tagIds) {
    //     if (id == primaryID) continue;
    //     System.out.println(primaryID);
    //     inputs.tagIds[i] = id;
    //     i++;
    //   }
    // }
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i] = id;
      i++;
    }
  }

  /** Parses the robot field pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  @Override
  public PhysicalJoint getBaseJoint() {
    return baseJoint;
  }

  @Override
  public Transform3d getMountingOffset() {
    return mountingOffset;
  }
}
