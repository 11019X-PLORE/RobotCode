// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis.controllers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.PoseEstimator;
/** Add your docs here. */
public class AimController {
    public PIDController aimingController;
    PoseEstimator m_poseEstimator;
    Rotation2d desireRotation=new Rotation2d();
    double desireAngularVelocity=0;
    double tolerance=Math.PI;
    public double currentDesireAngularSpeed=0;
    public AimController(PoseEstimator poseEstimator){
        this.m_poseEstimator=poseEstimator;
        aimingController=new PIDController(Constants.AutoConstants.kPAimingController,
                                                                0,
                                                                Constants.AutoConstants.kDAimingController
                                                                );
        aimingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public void setGoal(Rotation2d desireRotation2d,double desireAngularSpeed){
        this.desireRotation=desireRotation2d;
        this.desireAngularVelocity=desireAngularSpeed;
    }
    public void setGoal(Rotation2d desireRotation2d,double desireAngularSpeed,double tolerance){
        this.desireRotation=desireRotation2d;
        this.desireAngularVelocity=desireAngularSpeed;
        this.tolerance=tolerance;
    }
    public void setTolerance(double tolerance){
        this.tolerance=tolerance;
    }
    public void addGoal(){

    }
    public void setConstraints(){

    }

    public double getAngularSpeeds(){
        Pose2d currentPose=m_poseEstimator.getEstimatedPosition();
        double rotation=-aimingController.calculate(desireRotation.minus(currentPose.getRotation()).getRadians())+desireAngularVelocity;
        return rotation;
    }
    public boolean atGaol(){
        boolean atRotation=Math.abs(aimingController.getPositionError())<Math.abs(tolerance);
        return atRotation;
    }
    public double getError(){
        return aimingController.getPositionError();
    }
   
}
