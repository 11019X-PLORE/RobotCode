// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Shooter;

public class Turret extends SubsystemBase {
  private TalonFX turretMotor = new TalonFX(Shooter.turretMotorID);
  private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
  private MotionMagicVoltage TurretMotionMagic = new MotionMagicVoltage(0);
  // private PositionDutyCycle turretPosDutyCycle=new PositionDutyCycle(0);

  public double desireAngle=0;
  /** Creates a new Turret. */
  public Turret() {
    configTurret();
  }

  public void configTurret(){
    turretMotorConfig.Feedback.SensorToMechanismRatio = Shooter.turretGearRatio;

    turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    turretMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;

     /* Open and Closed Loop Ramping */
    turretMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    turretMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod=0;

    turretMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    turretMotorConfig.Slot0.kP = Shooter.turretKP;
    turretMotorConfig.Slot0.kI = Shooter.turretKI;
    turretMotorConfig.Slot0.kD = Shooter.turretKD;
    turretMotorConfig.Slot0.kV = Shooter.turretKV;
    turretMotorConfig.Slot0.kS = Shooter.turretKS;
    turretMotorConfig.Slot0.kA = Shooter.turretKA;
    turretMotorConfig.Slot0.kG = Shooter.turretKG;
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = Shooter.turretAcceleration;
    turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Shooter.turretVelocity;
    turretMotorConfig.MotionMagic.MotionMagicJerk = 0.0;

    turretMotor.getConfigurator().apply(turretMotorConfig);
    resetTurretPosition();
  }
  public double getTurretPosition(){
    return turretMotor.getPosition().getValueAsDouble();
  }
  public double getTurretAngle(){
    return getTurretPosition()*2*Math.PI;
  }
  public void resetTurretPosition(){
    turretMotor.getConfigurator().setPosition(Constants.Shooter.turretInitialAngle/(2*Math.PI));
  }
  public void stopTurret(){
    turretMotor.stopMotor();
  }
  public void TurretControl(double position){
    TurretMotionMagic.Position = position;
    // turretPosDutyCycle.Position=position;
    turretMotor.setControl(TurretMotionMagic.withFeedForward(Constants.Shooter.turretKG*Math.cos(getTurretAngle())));  
  }
  public void setTurretAngle(double theta){
    desireAngle = Math.min(
      Math.max(Constants.Shooter.turretMinimumAngle, theta)
      ,Constants.Shooter.turretMaximumAngle);
    TurretControl(desireAngle/(Math.PI*2));
  }
  public boolean atDesireAngle(){
    return Math.abs(desireAngle-getTurretAngle())<Constants.Shooter.turretTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("atDesireAngle()", atDesireAngle());
    // SmartDashboard.putNumber("turretAngle", getTurretAngle()*180/Math.PI);
  }
}
