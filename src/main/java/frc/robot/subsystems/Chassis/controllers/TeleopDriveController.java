// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis.controllers;

import java.security.Principal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Chassis.PoseEstimator;

/** Add your docs here. */
public class TeleopDriveController {

    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    PoseEstimator m_poseEstimator;
    public Translation2d inputDesireVelocity=new Translation2d(0,0);
    public double inputRotation=0;
    public TeleopDriveController(PoseEstimator poseEstimator){
        this.m_poseEstimator=poseEstimator;
    }

    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;
    public void updateControllerInput(double x, double y, double omega){
        controllerX=MathUtil.applyDeadband(x, Constants.OIConstants.stickDeadband);
        controllerY=MathUtil.applyDeadband(y, Constants.OIConstants.stickDeadband);
        controllerOmega=MathUtil.applyDeadband(omega, Constants.OIConstants.stickDeadband);
        // controllerX = x;
        // controllerY = y;
        // controllerOmega = omega;
        inputDesireVelocity=new Translation2d(controllerX, controllerY).times(Constants.Swerve.maxSpeed);
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red){
				inputDesireVelocity=inputDesireVelocity.rotateBy(new Rotation2d(Math.PI));
            }
        }
        
        inputRotation=controllerOmega* Math.max(Constants.Swerve.stationaryAngularVelocity,0);
    }

    public ChassisSpeeds getDesireSpeeds(){
        Translation2d desireVelocity=inputDesireVelocity;
        double rotation=inputRotation;
        return new ChassisSpeeds(
            desireVelocity.getX(), 
            desireVelocity.getY(), 
            rotation);
    }
    
}
