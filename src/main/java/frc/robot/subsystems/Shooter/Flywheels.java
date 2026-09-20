// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Flywheels extends SubsystemBase {
  /** Creates a new Flywheels. */
  private TalonFX topFlywheel = new TalonFX(Constants.Shooter.topFlywheelMotorID);
  private TalonFX bottomFlywheel = new TalonFX(Constants.Shooter.bottomFlywheelLeftMotorID);
  private TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

  private VelocityVoltage shootingVelDutycycle = new VelocityVoltage(0);

  public double desireVelocity=0;

  

  public Flywheels() {
    configFlywheels();
  }
  private void configFlywheels(){
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    flywheelConfig.Feedback.SensorToMechanismRatio=Constants.Shooter.flywheelGearRatio;

    /* Current Limiting */
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 20;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

     /* Open and Closed Loop Ramping */
    flywheelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    flywheelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    flywheelConfig.Slot0.kP = Constants.Shooter.flyWheelKP;
    flywheelConfig.Slot0.kI = Constants.Shooter.flyWheelKI;
    flywheelConfig.Slot0.kD = Constants.Shooter.flyWheelKD;
    flywheelConfig.Slot0.kV = Constants.Shooter.flyWheelKV;
    flywheelConfig.Slot0.kS = Constants.Shooter.flyWheelKS;
    flywheelConfig.Slot0.kA = Constants.Shooter.flyWheelKA;
    

    topFlywheel.getConfigurator().apply(flywheelConfig);    
    bottomFlywheel.getConfigurator().apply(flywheelConfig);
  }

  private void flywheelControl(double velocity){
    shootingVelDutycycle.Velocity = velocity;
    bottomFlywheel.setControl(shootingVelDutycycle);
    topFlywheel.setControl(new Follower(bottomFlywheel.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void setShootingVelocity(double velocity){
    desireVelocity=velocity;
    flywheelControl(Conversions.MPSToRPS(desireVelocity, Constants.Shooter.flywheelDiameter*Math.PI));
  }
  
  public void stopflywheel(){
    topFlywheel.stopMotor();
    bottomFlywheel.stopMotor();
  }

  //待改
  public double getShootingVelocity(){
    return Conversions.RPSToMPS(topFlywheel.getVelocity().getValueAsDouble(), Constants.Shooter.flywheelDiameter*Math.PI);
  }
  public boolean atDesireSpeed(){
    return Math.abs(getShootingVelocity()-desireVelocity)<Constants.Shooter.flywheelTolerance;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("shootingSpeed",getShootingVelocity());
    // SmartDashboard.putBoolean("atDesireSpeed",atDesireSpeed());
    // SmartDashboard.putNumber("atDesireSpeedError",Math.abs(getShootingVelocity()-desireVelocity));
    // SmartDashboard.putNumber("atDesireSpeeddesireVelocityError",Math.abs(desireVelocity));
  }
}
