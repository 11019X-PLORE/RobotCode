package frc.robot.subsystems.Chassis.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
/** Add your docs here. */
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Chassis.PoseEstimator;
import us.hebi.quickbuf.Descriptors.Descriptor;
public class AutoAlignController {
    public PIDController aimingController;
    public PIDController aligningController;
    PoseEstimator m_poseEstimator;
    public Pose2d desirePose=new Pose2d(0,0,new Rotation2d(0));
    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;
    public AutoAlignController(PoseEstimator poseEstimator){
        this.m_poseEstimator=poseEstimator;
        aligningController=new PIDController(Constants.AutoConstants.kPAligningController,
                                                            0,
                                                            Constants.AutoConstants.kDAligningController);
   
        aimingController=new PIDController(Constants.AutoConstants.kPAimingController,
                                                                0,
                                                                Constants.AutoConstants.kDAimingController
                                                                );
    }
    public void setGoal(Pose2d desirePose2d){
        this.desirePose=desirePose2d;
    }
    public void setGoal(Pose2d desirePose2d,DriverStation.Alliance allianceColor){
        this.desirePose=inversePose2dUsingAlliance(desirePose2d, allianceColor);
    }
    public void addGoal(){

    }
    public void setConstraints(){

    }

    public ChassisSpeeds getDesireSpeeds(){
        Pose2d currentPose=m_poseEstimator.getEstimatedPosition();
        Translation2d deltaTranslation=desirePose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d deltaRotation=desirePose.getRotation().minus(currentPose.getRotation());
        Translation2d desireVelocity=new Translation2d(-aligningController.calculate(deltaTranslation.getNorm()),0).rotateBy(deltaTranslation.getAngle());
        double rotation=-aimingController.calculate(deltaRotation.getRadians());
        return new ChassisSpeeds(
            desireVelocity.getX(), 
            desireVelocity.getY(), 
            rotation);
    }
    public boolean atGaol(){
        boolean atSetpoint=Math.abs(aligningController.getPositionError())<Constants.AutoConstants.translationTolerance&&
                    Math.abs(aimingController.getPositionError())<Constants.AutoConstants.aimTolerance;
        return atSetpoint;
    }
    public Pose2d inversePose2dUsingAlliance(Pose2d pose,DriverStation.Alliance allianceColor){
        if(allianceColor!=getAlliance()){
            return new Pose2d(Constants.fieldConstants.fieldSize.getX()-pose.getX(),pose.getY(),new Rotation2d(Math.PI).minus(pose.getRotation()));
        }
        return pose;
    }
    public DriverStation.Alliance getAlliance(){
        try{
            return DriverStation.getAlliance().get();
        }catch(Exception e){
            return DriverStation.Alliance.Red;
        }
    }
    
}
