// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Carriage.CarriageSystem;
import frc.robot.subsystems.Carriage.CarriageSystem.carriageState;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSystem;
import frc.robot.subsystems.Intake.IntakeSystem.intakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class ShooterSystem extends SubsystemBase {
  DriveSubsystem m_swerve;
  Flywheels m_flywheels;
  Turret m_turret;
  Triggershooting m_trigger;
  LEDSubsystem m_LED;
  IntakeSystem m_intake;
  CarriageSystem m_carriage;

  boolean ready=false;
  public boolean enableOrin=true;

  public enum shooterState{
    AUTO,
    PASSING,
    AUTONOMOUSPREIOD,
    PREPARING,
    SPIT,
    STOP,
    DEFAULT,
    DEFAULTSHOOT,
    Source,
    FASTSHOOT,
    AUTOPOINT,
    TurretLowPosition,
    Turret
  }

  shooterState currentShooterState=shooterState.STOP;
  /** Creates a new Shooter. */
  public ShooterSystem(DriveSubsystem swerve,Flywheels flywheels,Turret turret,Triggershooting trigger,LEDSubsystem LED,IntakeSystem intake,CarriageSystem carriage) {
    this.m_swerve=swerve;
    this.m_flywheels=flywheels;
    this.m_turret=turret;
    this.m_trigger=trigger;
    this.m_LED=LED;
    this.m_intake=intake;
    this.m_carriage=carriage;
  }

  public double getStationaryShootingSpeed(double distance){
    return 4.125*distance+10.0;
  }
  public Translation2d getStationaryPassingSpeed(double distance){
    return new Translation2d(distance/(2*Math.sqrt(2*Constants.fieldConstants.passingHeight/(-Constants.physicalConstants.g))),
      Math.sqrt(2*(-Constants.physicalConstants.g)*Constants.fieldConstants.passingHeight));
  }
  public Translation2d getLaunchingPose(){
    return Constants.Shooter.turretPivotTranslation.plus(
      new Translation2d(Constants.Shooter.turretRadius,0).rotateBy(new Rotation2d(Math.PI-m_turret.getTurretAngle())));
  }
  public double getShootingAngle(double dx,double dy,double v){
    double g=Constants.physicalConstants.g;
    // double angle=Math.atan((v*v-Math.sqrt(Math.pow(v,4))-(g*g*dx*dx)-(2*g*v*v*(-dy)))/(g*(-dx)));
    double angle=Math.atan((Math.pow(v,2)-Math.sqrt(Math.pow(v,4)-(Math.pow(g,2)*Math.pow(dx,2))-(2*g*Math.pow(v,2)*(-dy))))/(g*(-dx)));
    // SmartDashboard.putNumber("dx", dx);
    angle+=Math.atan(quadratic(dx,getStandardQuadraticFunction(1.195, -0.04, 3.50, -0.03, 5.25, 0.0763)));
    return angle;
  }
  // public double getAutoShootingAngle(double dx,double dy,double v){
  //   double g=Constants.physicalConstants.g;
  //   // double angle=Math.atan((v*v-Math.sqrt(Math.pow(v,4))-(g*g*dx*dx)-(2*g*v*v*(-dy)))/(g*(-dx)));
  //   double angle=Math.atan((Math.pow(v,2)-Math.sqrt(Math.pow(v,4)-(Math.pow(g,2)*Math.pow(dx,2))-(2*g*Math.pow(v,2)*(-dy))))/(g*(-dx)));
  //   SmartDashboard.putNumber("dx", dx);
  //   angle+=Math.atan(quadratic(dx,getStandardQuadraticFunction(1.195, 0.0, 3.50, 0.0, 5.25, 0.0)));
  //   return angle;
  // }
  private List<Double> getStandardQuadraticFunction(double x1,double y1,double x2,double y2,double x3,double y3){
    double a=(((y1-y2)/(x1-x2))-((y2-y3)/(x2-x3)))/((x1-x3));
    double b=((y2-y3)-a*(x2*x2-x3*x3))/(x2-x3);
    double c=y3-a*x3*x3-b*x3;
    return List.of(a,b,c);
  }
  public double quadratic(double x,List<Double> fittingFunctionCoefficients){
    return x*x*fittingFunctionCoefficients.get(0)+x*fittingFunctionCoefficients.get(1)+fittingFunctionCoefficients.get(2);
  }

  public Translation2d getTranslationToTarget(Translation2d targetAtBlueAlliance){
    Translation2d targetTranslation=m_swerve.inverseTranslationUsingAlliance(targetAtBlueAlliance, DriverStation.Alliance.Blue);
    return targetTranslation.minus(m_swerve.getPoseWithTime(Timer.getFPGATimestamp()+Constants.Shooter.predictTime).getTranslation());
  }

  public Translation2d getRobotVelocity(){
    ChassisSpeeds chassisSpeeds=m_swerve.getFieldRelativeSpeeds();
    return new Translation2d(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond);
  }
  
  public Translation3d getShootingVelocity(){
    Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
    Translation2d launchingPose= getLaunchingPose();
    Translation2d robotVelocity=getRobotVelocity();
    double deltaY=Constants.fieldConstants.SpeakerTranslation.getZ()-launchingPose.getY();
    // SmartDashboard.putNumber("dy", deltaY);
    double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
    double stationaryShootingSpeed=getStationaryShootingSpeed(distanceToTraget);
    
    double stationaryShootingPitch=getShootingAngle(distanceToTraget, deltaY, stationaryShootingSpeed);
    SmartDashboard.putNumber("dx", distanceToTraget);
    SmartDashboard.putNumber("shootringSpeed", stationaryShootingSpeed);
    SmartDashboard.putNumber("dy", deltaY);
    SmartDashboard.putNumber("pitch",stationaryShootingPitch*180/Math.PI);
    double stationaryShootingDirection=translationToTarget.getAngle().getRadians();
    Translation3d stationaryShootingVector= new Translation3d(stationaryShootingSpeed,0,0).rotateBy(
      new Rotation3d(0,-stationaryShootingPitch,stationaryShootingDirection));
    stationaryShootingVector=stationaryShootingVector.minus(new Translation3d(robotVelocity.getX(),robotVelocity.getY(),0));
    return stationaryShootingVector;
  }
  public Translation3d getPassingVelocity(){
    Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.passingTarget);
    Translation2d launchingPose= getLaunchingPose();
    Translation2d robotVelocity=getRobotVelocity();
    double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
    Translation2d stationaryPassingVelocity=getStationaryPassingSpeed(distanceToTraget);
    // SmartDashboard.putString("passing v", stationaryPassingVelocity.toString());
    double stationaryPassingPitch=stationaryPassingVelocity.getAngle().getRadians();
    double stationaryPassingDirection=translationToTarget.getAngle().getRadians();
    Translation3d stationaryPassingVector= new Translation3d(stationaryPassingVelocity.getNorm()*1.5,0,0).rotateBy(
      new Rotation3d(0,-stationaryPassingPitch,stationaryPassingDirection));
    return stationaryPassingVector.minus(new Translation3d(robotVelocity.getX(),robotVelocity.getY(),0));
  }
  // public Translation3d getAutoShootingVelocity(){
  //   Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
  //   Translation2d launchingPose= getLaunchingPose();
  //   Translation2d robotVelocity=getRobotVelocity();
  //   double deltaY=Constants.fieldConstants.SpeakerTranslation.getZ()-launchingPose.getY();
  //   SmartDashboard.putNumber("dy", deltaY);
  //   double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
  //   double stationaryShootingSpeed=getStationaryShootingSpeed(distanceToTraget);
    
  //   double stationaryShootingPitch=getAutoShootingAngle(distanceToTraget, deltaY, stationaryShootingSpeed);
  //   SmartDashboard.putNumber("dx", distanceToTraget);
  //   double stationaryShootingDirection=translationToTarget.getAngle().getRadians();
  //   Translation3d stationaryShootingVector= new Translation3d(stationaryShootingSpeed,0,0).rotateBy(
  //     new Rotation3d(0,-stationaryShootingPitch,stationaryShootingDirection));
  //   stationaryShootingVector=stationaryShootingVector.minus(new Translation3d(robotVelocity.getX(),robotVelocity.getY(),0));
  //   return stationaryShootingVector;
  // }
  public double getAutoPreparingSpeed(){
    double distanceToSpeaker=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d()).getNorm();
    double shootingSpeed=getShootingVelocity().getNorm();
    if(m_trigger.sensorDetected()){
      if(distanceToSpeaker<Constants.Shooter.preparingRange){
        return shootingSpeed;
      }else{
        return Constants.Shooter.defaultHighSpeed;
      }
    }else{
      if(distanceToSpeaker<Constants.Shooter.preparingRange){
        return Constants.Shooter.defaultHighSpeed;
      }else{
        return Constants.Shooter.defaultLowSpeed;
      }
    }
  }
  public double getDesireShootingAngularSpeed(Translation2d Target){
    Translation2d translationToTarget=getTranslationToTarget(Target);
    Translation2d robotVelocity=getRobotVelocity();
    double verticalVeloctiy=robotVelocity.rotateBy(translationToTarget.getAngle().times(-1)).getY();
    return verticalVeloctiy/translationToTarget.getNorm();
  }
  public void setState(shooterState state){
    currentShooterState=state;
  }
  public void orinSwitch(){
    if(enableOrin){
      enableOrin=false;
    }else{
      enableOrin=true;
    }
  }
  public boolean getorinSwitchstate(){
    return enableOrin;
  }
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Drive", m_swerve.atAimTarget());
    SmartDashboard.putBoolean("orinSwitch", enableOrin);
    // SmartDashboard.putNumber("distance",);
    switch (currentShooterState) {
      case STOP->{
       m_flywheels.stopflywheel();
       m_turret.stopTurret(); 
       m_trigger.stopTrigger();
      }
      case AUTO->{
        Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        Translation2d launchingPose= getLaunchingPose();
        Translation3d shootingVelocity=getShootingVelocity();
        double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
        if(Math.abs(distanceToTraget)<4.3){
          m_flywheels.setShootingVelocity(Shooter.defaultHighSpeed);
        }else{
          m_flywheels.setShootingVelocity(shootingVelocity.getNorm());
        }
        double desireAngularSpeed=getDesireShootingAngularSpeed(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        m_turret.setTurretAngle(new Translation2d(shootingVelocity.toTranslation2d().getNorm(),shootingVelocity.getZ()).getAngle().getRadians());
        m_swerve.setAimTarget(shootingVelocity.toTranslation2d().getAngle().plus(new Rotation2d(Math.PI)).plus(new Rotation2d(Shooter.errorDriveRadius*Math.PI/180)),
          desireAngularSpeed,
          Math.atan((Constants.fieldConstants.speakerWidth/2)/
            getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d()).getNorm()));
        ready=m_turret.atDesireAngle()&&m_flywheels.atDesireSpeed()&&m_swerve.atAimTarget();
        if(ready){
          m_trigger.settriggerVelocity(Shooter.ShootingVel);
        }else{
          m_trigger.stopTrigger();
        }
        // m_LED.update(LEDState.AUTO);
      }
      case AUTONOMOUSPREIOD->{
        Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        Translation2d launchingPose= getLaunchingPose();
        Translation3d shootingVelocity=getShootingVelocity();
        double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
        if(Math.abs(distanceToTraget)<4.3){
          m_flywheels.setShootingVelocity(Shooter.defaultHighSpeed);
        }else{
          m_flywheels.setShootingVelocity(shootingVelocity.getNorm());
        }
        // m_flywheels.setShootingVelocity(shootingVelocity.getNorm());
        double desireAngularSpeed=getDesireShootingAngularSpeed(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        m_turret.setTurretAngle(new Translation2d(shootingVelocity.toTranslation2d().getNorm(),shootingVelocity.getZ()).getAngle().getRadians());
        m_swerve.setAimTarget(shootingVelocity.toTranslation2d().getAngle().plus(new Rotation2d(Math.PI)).plus(new Rotation2d(Shooter.errorDriveRadius*Math.PI/180)),
          desireAngularSpeed,
          Math.atan((Constants.fieldConstants.speakerWidth/2)/
            getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d()).getNorm()));
        ready=m_turret.atDesireAngle()&&m_flywheels.atDesireSpeed()&&m_swerve.atAimTarget();
        if(ready){
          m_trigger.settriggerVelocity(Shooter.ShootingVel);
        }else{
          m_trigger.stopTrigger();
        }
      }
      case PREPARING->{
        Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        Translation2d launchingPose= getLaunchingPose();
        Translation3d shootingVelocity=getShootingVelocity();
        double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
        if(Math.abs(distanceToTraget)<4.3){
          m_flywheels.setShootingVelocity(25.0);
        }else{
          m_flywheels.setShootingVelocity(shootingVelocity.getNorm());
        }
        // if(Math.abs(distanceToTraget)<3.5){
        //   m_turret.setTurretAngle(Shooter.turretAutoAngle);
        // }else{
        m_turret.setTurretAngle(new Translation2d(shootingVelocity.toTranslation2d().getNorm(),shootingVelocity.getZ()).getAngle().getRadians()+Shooter.turretAutoAngleError);
        // }
      }
      case PASSING->{
        Translation3d passingVelocity=getPassingVelocity();
        double desireAngularSpeed=getDesireShootingAngularSpeed(Constants.fieldConstants.passingTarget);
        m_flywheels.setShootingVelocity(Shooter.PassSpeed);
        m_turret.setTurretAngle(Constants.Shooter.turretDefaultAngle);
        m_swerve.setAimTarget(passingVelocity.toTranslation2d().getAngle().plus(new Rotation2d(Math.PI)).plus(new Rotation2d(Shooter.errorDrivePassRadius*Math.PI/180)), desireAngularSpeed,Math.PI/18);
        ready=m_turret.atDesireAngle()&&m_flywheels.atDesireSpeed()&&m_swerve.atAimTarget();
        if(ready){
          m_trigger.settriggerVelocity(Shooter.ShootingVel);
        }else{
          m_trigger.stopTrigger();
        }
      }
      case DEFAULTSHOOT->{
        m_flywheels.setShootingVelocity(Constants.Shooter.defaultLowSpeed);
        m_turret.setTurretAngle(Constants.Shooter.turretAutoDefaultAngle);
        ready=m_turret.atDesireAngle()&&m_flywheels.atDesireSpeed();
        if(ready){
          m_trigger.settriggerVelocity(Shooter.ShootingVel);
        }else{
          m_trigger.stopTrigger();
        }
      }
      case DEFAULT->{
        m_flywheels.setShootingVelocity(Constants.Shooter.defaultLowSpeed);
        m_turret.setTurretAngle(Constants.Shooter.turretDefaultAngle);
      }
      case TurretLowPosition->{
        m_turret.setTurretAngle(Constants.Shooter.TurretLowPosition);
      }
      case Turret->{
        m_turret.setTurretAngle(Constants.Shooter.turretDefaultAngle);
      }
      case FASTSHOOT->{
        Translation2d translationToTarget=getTranslationToTarget(Constants.fieldConstants.SpeakerTranslation.toTranslation2d());
        Translation2d launchingPose= getLaunchingPose();
        Translation3d shootingVelocity=getShootingVelocity();
        double distanceToTraget=translationToTarget.getNorm()+launchingPose.getX();
        if(Math.abs(distanceToTraget)<4.3){
          m_flywheels.setShootingVelocity(25.0);
        }else{
          m_flywheels.setShootingVelocity(shootingVelocity.getNorm());
        }
        // if(Math.abs(distanceToTraget)<2.3){
        //   m_turret.setTurretAngle(Shooter.turretAutoAngle);
        // }else{
        m_turret.setTurretAngle(new Translation2d(shootingVelocity.toTranslation2d().getNorm(),shootingVelocity.getZ()).getAngle().getRadians()+Shooter.turretAutoAngleError);
        // }
        m_trigger.settriggerVelocity(Shooter.ShootingVel);
        m_intake.setState(intakeState.SHOOTING);
        m_carriage.setState(carriageState.SHOOTING);
      }
      case AUTOPOINT->{
        m_flywheels.setShootingVelocity(Constants.Shooter.defaultLowSpeed);
        m_turret.setTurretAngle(Constants.Shooter.turretDefaultAngle);
        ready=m_turret.atDesireAngle()&&m_flywheels.atDesireSpeed();
        if(ready){
          m_trigger.settriggerVelocity(Shooter.ShootingVel);
          m_intake.setState(intakeState.SHOOTING);
          m_carriage.setState(carriageState.SHOOTING);
        }else{
          m_trigger.stopTrigger();
          m_intake.setState(intakeState.STOP);
          m_carriage.setState(carriageState.STOP);
        }
      }
      case SPIT->{
        m_flywheels.setShootingVelocity(Constants.Shooter.defaultLowSpeed);
        m_turret.setTurretAngle(Constants.Shooter.turretDefaultAngle);
        m_trigger.settriggerVelocity(Shooter.FowardVel);
      }
      case Source->{
        if(m_trigger.sensorDetected()){
          m_flywheels.stopflywheel();
          m_turret.setTurretAngle(Constants.Shooter.turretsourceAngle);
          m_trigger.stopTrigger();
        }else{
          m_flywheels.setShootingVelocity(Shooter.sourceSpeed);
          m_turret.setTurretAngle(Constants.Shooter.turretsourceAngle);
          m_trigger.settriggerVelocity(Shooter.BackwardVel);
        }
      }
      default->{

      }
    }
  }
}
