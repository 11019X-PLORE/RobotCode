package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();
  public double goal = 0.0;
  public boolean pidMode = true;
  public boolean neutralMode_brake = true;

  // SysIdRoutine start
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_distance = Radian.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                setVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("arm-all")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(
                        m_distance.mut_replace(getAngleRads() - Math.toRadians(20), Radians))
                    .angularVelocity(m_velocity.mut_replace(getOmegaRadPerSec(), RadiansPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  // SysIdRoutine end

  public Arm(ArmIO io) {
    this.io = io;
    io.setArmPID(
        ArmConstants.kp,
        ArmConstants.Ki,
        ArmConstants.Ks,
        ArmConstants.Kd,
        ArmConstants.Kv,
        ArmConstants.Ka,
        ArmConstants.Kg,
        ArmConstants.acceleration,
        ArmConstants.velocity);
  }

  public void runGoal(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    goal = MathUtil.clamp(angleRads, ArmConstants.armMinimumAngle, ArmConstants.armMaximumAngle);
    // SmartDashboard.putNumber("torqueflag1",torque);
    io.runPositionSetpoint(
        goal, omegaRadPerSec, alphaRadsPerSecSquared, torque); // / TorqueNumber.getAsDouble());
  }

  public void runArmPosition(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    goal = MathUtil.clamp(angleRads, ArmConstants.armMinimumAngle, ArmConstants.armMaximumAngle);
    io.runMotionMagicArmPosition(goal, omegaRadPerSec, alphaRadsPerSecSquared, torque);
  }

  public void runClimberPosition(double angleRads) {
    goal = MathUtil.clamp(angleRads, ArmConstants.armMinimumAngle, ArmConstants.armMaximumAngle);
    io.runMotionMagicClimberPosition(goal);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void switchBrake() {
    neutralMode_brake = !neutralMode_brake;
    io.switchBrake(!neutralMode_brake);
  }

  public void stop() {
    io.stop();
  }

  public double getOmegaRadPerSec() {
    return inputs.armOmegaRadPerSec_LF;
  }

  public double getAlphaRadsPerSecSquared() {
    return inputs.armAlphaRadsPerSecSquared_LF;
  }

  public double getAngleRads() {
    return inputs.armPositionRads_LF;
  }

  public double getvolts() {
    return inputs.armAppliedVolts_LF;
  }

  public boolean atGoal() {
    return Math.abs(goal - getAngleRads()) < ArmConstants.armTolerance;
  }

  public void setVoltage(Voltage volt) {
    io.setVoltage(volt);
  }

  public double get() {
    return io.get();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
