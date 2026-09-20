// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Carriage.Feeder;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSystem;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.objectTracker;

public class BlackholePilot extends SubsystemBase {
  DriveSubsystem m_DriveSubsystem;
  VisionIO m_visionIO;
  objectTracker m_objectTracker;
  Feeder m_feeder;
  /** Creates a new BlackholePilot. */
  public BlackholePilot(VisionIO visionIO,DriveSubsystem driveSubsystem,objectTracker objectTracker, Feeder feeder) {
    this.m_visionIO=visionIO;
    this.m_DriveSubsystem=driveSubsystem;
    this.m_objectTracker=objectTracker;
    this.m_feeder=feeder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Translation2d othersPose=m_objectTracker.mostDangerousRobot;
    if(othersPose == null){
      othersPose=new Translation2d(100,100);
    }

    m_visionIO.sendTrainInputs(m_DriveSubsystem.getPose(),
      m_DriveSubsystem.getFieldRelativeSpeeds(),
      othersPose.minus(m_DriveSubsystem.getPose().getTranslation()),
      m_feeder.sensorintakeDetected()
    );
    m_visionIO.sendActions(m_DriveSubsystem.getDriverDesireSpeeds());
  }
}
