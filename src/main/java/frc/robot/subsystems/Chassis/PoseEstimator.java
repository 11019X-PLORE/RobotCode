// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PoseEstimator {
    public TimeInterpolatableBuffer<Double> turretYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public TimeInterpolatableBuffer<Double> gyroYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public NavigableMap<Double, Twist2d> odometryUpdates= new TreeMap<>();
    public NavigableMap<Double, Pose2d> robotPoseBuffer = new TreeMap<>();
    public Pose2d visionPose = new Pose2d();

    private Rotation2d lastGyroYaw=new Rotation2d(0);
    private SwerveModulePosition[] lastModulePositions;
    private Pose2d estimatedPose;
    private double errorInEstimate;
    private SwerveDriveKinematics kinematics;
    private boolean updatingVision=false;

    public PoseEstimator(Pose2d initialPose,double initialErrorInEstimate,SwerveDriveKinematics kinematics){
        estimatedPose=initialPose;
        this.kinematics=kinematics;
        this.errorInEstimate=initialErrorInEstimate;
    }

    public void updateDrive(Rotation2d gyroYaw,SwerveModulePosition[] modulePositions,double processNoisePerMeter){
        if(!updatingVision){
            SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[modulePositions.length];
            if(lastModulePositions==null){
                lastModulePositions=modulePositions;
            }
            for (int i = 0; i < modulePositions.length; i++) {
                wheelDeltas[i] =
                    new SwerveModulePosition(
                        (modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters),
                        modulePositions[i].angle);
            }
            Twist2d twist = kinematics.toTwist2d(wheelDeltas);
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());

            lastGyroYaw = gyroYaw;
            lastModulePositions=modulePositions;
            estimatedPose=estimatedPose.exp(twist);
            errorInEstimate=errorInEstimate+processNoisePerMeter*new Translation3d(twist.dx,twist.dy,twist.dtheta).getNorm();
            odometryUpdates.put(Timer.getFPGATimestamp(), twist);
            while (odometryUpdates.firstKey()<Timer.getFPGATimestamp() -2) {
                    odometryUpdates.pollFirstEntry();
            }
            robotPoseBuffer.put(Timer.getFPGATimestamp(), estimatedPose);
            while (robotPoseBuffer.firstKey()<Timer.getFPGATimestamp() -2) {
                robotPoseBuffer.pollFirstEntry();
            }
        }

    }

    public Twist2d getTotalOdometryChange(double startTime, double endTime) {
        // Ensure that startTime is not in the future
        if (startTime > endTime) {
            return new Twist2d(0,0,0);
        }

        // Sum the twists between the startTime and endTime
        Twist2d totalChange = new Twist2d(0, 0, 0);
        NavigableMap.Entry<Double, Twist2d> startEntry = odometryUpdates.ceilingEntry(startTime);

        // If startEntry is null, it means no updates exist after startTime
        if (startEntry == null) {
            return totalChange;
        }

        for (NavigableMap.Entry<Double, Twist2d> entry : odometryUpdates.tailMap(startEntry.getKey()).entrySet()) {
            if (entry.getKey() > endTime) {
                break;
            }
            totalChange = new Twist2d(
                    totalChange.dx + entry.getValue().dx,
                    totalChange.dy + entry.getValue().dy,
                    totalChange.dtheta + entry.getValue().dtheta
            );
        }

        return totalChange;
    }
    public void addErrorInEstimate(double newError){
        errorInEstimate+=newError;
    }

    public void updateVisionWithTime(Pose2d pose,double errorInMeasurement, Rotation2d gyroYaw ,double timeStamp){
        if(timeStamp>0){
            try{
                pose=pose.exp(getTotalOdometryChange(timeStamp,Timer.getFPGATimestamp()));
                // SmartDashboard.putString("do",getTotalOdometryChange(timeStamp,Timer.getFPGATimestamp()).toString());
                pose=new Pose2d(pose.getX(),pose.getY(),estimatedPose.getRotation());
                double kalmanGain=errorInEstimate/(Math.abs(errorInMeasurement)+errorInEstimate);
                // estimatedPose=estimatedPose.transformBy(new Transform2d(estimatedPose,pose).times(kalmanGain));
                estimatedPose=estimatedPose.transformBy(new Transform2d(estimatedPose,pose).times(kalmanGain));
                errorInEstimate=Math.abs((1-kalmanGain)*(errorInEstimate));
            }catch(ConcurrentModificationException e){

            }
        }
    }

    public boolean poseIsValid(TimestampedVisionUpdate visionPose,Rotation2d gyro, double gyroTolerance,double errorTolerance,double heightTolerance){
        if(Math.abs(visionPose.pose.toPose2d().getRotation().getRadians()-gyro.getRadians())<gyroTolerance&&
            errorTolerance>visionPose.errorInMeasurement&&
            Math.abs(visionPose.pose.getZ())<heightTolerance&&
            Math.abs(visionPose.pose.getRotation().getX())<gyroTolerance&&
            Math.abs(visionPose.pose.getRotation().getX())<gyroTolerance){
            return true;
        }
        return false;
    }
    public void visionInput(List<TimestampedVisionUpdate> visionPoses,Rotation2d gyroYaw,double gyroYawTolerance,double errorTolerance,double heightTolerance){
        for(TimestampedVisionUpdate visionPose : visionPoses){
            Pose2d estimatedPose=getRobotPoseWithTime(visionPose.timestamp);
            if (estimatedPose!=null){
                if(poseIsValid(visionPose, estimatedPose.getRotation(), gyroYawTolerance,errorTolerance,heightTolerance)){
                    updatingVision=true;
                    updateVisionWithTime(visionPose.pose.toPose2d(),visionPose.errorInMeasurement, gyroYaw, visionPose.timestamp);
                    updatingVision=false;
                }
            }
        }
    }
    public Pose2d getEstimatedPosition(){
        return estimatedPose;
    }
    public Pose2d getRobotPoseWithTime(double timestamp){
        try{
            if(robotPoseBuffer.firstKey()<timestamp){
                return robotPoseBuffer.lowerEntry(timestamp).getValue();
            }else{
                return null;
            }
        }catch(Exception e){
            return null;
        }
    }
    public void setPose(Pose2d pose) {
      estimatedPose=pose;
    }
    public void setRotation(Rotation2d rotation){
        estimatedPose=new Pose2d(estimatedPose.getX(),estimatedPose.getY(),rotation);
    }
    public static record TimestampedVisionUpdate(
      double timestamp, Pose3d pose, double errorInMeasurement) {}

}
