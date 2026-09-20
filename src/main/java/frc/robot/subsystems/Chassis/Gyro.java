// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  public static Pigeon2 gyro;
  public Gyro() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
  }
  public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }
  public void zeroHeading(){
    gyro.setYaw(0);
  }
  public void setHeading(Rotation2d heading){
    gyro.setYaw(heading.getDegrees());
  }
  public Translation3d getTotalAcceleration(){
    return new Translation3d(gyro.getAccelerationX().getValueAsDouble(),gyro.getAccelerationY().getValueAsDouble(),gyro.getAccelerationZ().getValueAsDouble());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("gyro", getGyroYaw().getDegrees());
  }
}
