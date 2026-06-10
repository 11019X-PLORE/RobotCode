package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.BlackholePlanner.Setpoint2d;
import frc.robot.util.BlackholePlanner.Trajectory2d;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class UpperSructure extends SubsystemBase {
  private final Arm arm;
  private final Elevator elevator;
  private final PS4Controller driver;

  private double armMass = 15.0; // 3., 2.85
  private double g = 9.799;
  private boolean addHeight = false;
  private boolean subHeight = false;
  private int level = 0;
  private final int MAX_VALUE = 3;
  private final int MIN_VALUE = 0;

  // 6 points traj
  private Trajectory2d I2L1Trajectory;
  private Trajectory2d I2L2Trajectory;
  private Trajectory2d I2L3Trajectory;
  private Trajectory2d I2L4Trajectory;
  private Trajectory2d I2STrajectory;
  private Trajectory2d S2L1Trajectory;
  private Trajectory2d S2L2Trajectory;
  private Trajectory2d S2L3Trajectory;
  private Trajectory2d S2L4Trajectory;
  private Trajectory2d S2ITrajectory;
  private Trajectory2d L12STrajectory;
  private Trajectory2d L12L2Trajectory;
  private Trajectory2d L12L3Trajectory;
  private Trajectory2d L12L4Trajectory;
  private Trajectory2d L12ITrajectory;
  private Trajectory2d L22ITrajectory;
  private Trajectory2d L22L1Trajectory;
  private Trajectory2d L22L3Trajectory;
  private Trajectory2d L22L4Trajectory;
  private Trajectory2d L22STrajectory;
  private Trajectory2d L22STTrajectory;
  private Trajectory2d L32ITrajectory;
  private Trajectory2d L32L1Trajectory;
  private Trajectory2d L32L2Trajectory;
  private Trajectory2d L32L4Trajectory;
  private Trajectory2d L32STrajectory;
  private Trajectory2d L42ITrajectory;
  private Trajectory2d L42L1Trajectory;
  private Trajectory2d L42L2Trajectory;
  private Trajectory2d L42L3Trajectory;
  private Trajectory2d L42STrajectory;
  private Trajectory2d ST2L3Trajectory;
  private Trajectory2d ST2L2Trajectory;
  private Trajectory2d ST2STrajectory;
  private Trajectory2d L32STTrajectory;

  public static double trajStartTime = 0.0;

  // Define state store
  private final Map<String, Trajectory2d> pathMap = new HashMap<>();

  public enum ArmElevatorState {
    Initial, //
    Start, // intake stop, arm stop, ele stop
    Stop, // intake stop, arm stop, ele stop
    ScoreL1, // intake as before, arm move to scorel1, ele move to scorel1
    ScoreL2, // intake as before, arm move to scorel2, ele move to scorel2
    ScoreL3, // intake as before, arm move to scorel3, ele move to scorel3
    ScoreL4, // intake as before, arm move to scorel4, ele move to scorel4
    StationIntake, // intake as before, arm move to station, ele move to station
  }

  public ArmElevatorState currentArmState;
  private ArmElevatorState previousArmState;
  private Trajectory2d currentPath;

  public UpperSructure(Arm arm, Elevator elevator, PS4Controller driver) {
    this.arm = arm;
    this.elevator = elevator;
    this.driver = driver;

    ST2L3Trajectory = new Trajectory2d("ST2L3", 1);
    ST2L2Trajectory = new Trajectory2d("ST2L2", 1);
    ST2STrajectory = new Trajectory2d("ST2S", 1);
    L32STTrajectory = new Trajectory2d("L32ST", 1);

    I2L1Trajectory = new Trajectory2d("I2L1", 1);
    I2L2Trajectory = new Trajectory2d("I2L2", 1);
    I2L3Trajectory = new Trajectory2d("I2L3", 1);
    I2L4Trajectory = new Trajectory2d("I2L4", 1);
    I2STrajectory = new Trajectory2d("I2S", 1);
    S2L1Trajectory = new Trajectory2d("S2L1", 1);
    S2L2Trajectory = new Trajectory2d("S2L2", 1);
    S2L3Trajectory = new Trajectory2d("S2L3", 1);
    S2L4Trajectory = new Trajectory2d("S2L4", 1);
    S2ITrajectory = new Trajectory2d("S2I", 1);
    L12STrajectory = new Trajectory2d("L12S", 1);
    L12L2Trajectory = new Trajectory2d("L12L2", 1);
    L12L3Trajectory = new Trajectory2d("L12L3", 1);
    L12L4Trajectory = new Trajectory2d("L12L4", 1);
    L12ITrajectory = new Trajectory2d("L12I", 1);
    L22ITrajectory = new Trajectory2d("L22I", 1);
    L22L1Trajectory = new Trajectory2d("L22L1", 1);
    L22L3Trajectory = new Trajectory2d("L22L3", 1);
    L22L4Trajectory = new Trajectory2d("L22L4", 1);
    L22STrajectory = new Trajectory2d("L22S", 1);
    L22STTrajectory = new Trajectory2d("L22ST", 1);
    L32ITrajectory = new Trajectory2d("L32I", 1);
    L32L1Trajectory = new Trajectory2d("L32L1", 1);
    L32L2Trajectory = new Trajectory2d("L32L2", 1);
    L32L4Trajectory = new Trajectory2d("L32L4", 1);
    L32STrajectory = new Trajectory2d("L32S", 1);
    L42ITrajectory = new Trajectory2d("L42I", 1);
    L42L1Trajectory = new Trajectory2d("L42L1", 1);
    L42L2Trajectory = new Trajectory2d("L42L2", 1);
    L42L3Trajectory = new Trajectory2d("L42L3", 1);
    L42STrajectory = new Trajectory2d("L42S", 1);

    // 6 points, L1,L2,L3,L4,I(initial), S(Station)
    pathMap.put(key(ArmElevatorState.Initial, ArmElevatorState.ScoreL1), I2L1Trajectory);
    pathMap.put(key(ArmElevatorState.Initial, ArmElevatorState.ScoreL2), I2L2Trajectory);
    pathMap.put(key(ArmElevatorState.Initial, ArmElevatorState.ScoreL3), I2L3Trajectory);
    // pathMap.put(key(ArmElevatorState.Initial, ArmElevatorState.ScoreL4), I2L4Trajectory);//Can't
    // use
    pathMap.put(key(ArmElevatorState.Initial, ArmElevatorState.StationIntake), I2STrajectory);
    pathMap.put(key(ArmElevatorState.StationIntake, ArmElevatorState.ScoreL1), S2L1Trajectory);
    pathMap.put(key(ArmElevatorState.StationIntake, ArmElevatorState.ScoreL2), S2L2Trajectory);
    pathMap.put(key(ArmElevatorState.StationIntake, ArmElevatorState.ScoreL3), S2L3Trajectory);
    // pathMap.put(key(ArmElevatorState.StationIntake,
    // ArmElevatorState.ScoreL4),S2L4Trajectory);//Can't use
    pathMap.put(key(ArmElevatorState.StationIntake, ArmElevatorState.Initial), S2ITrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL1, ArmElevatorState.StationIntake), L12STrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL1, ArmElevatorState.ScoreL2), L12L2Trajectory);
    pathMap.put(key(ArmElevatorState.ScoreL1, ArmElevatorState.ScoreL3), L12L3Trajectory);
    // pathMap.put(key(ArmElevatorState.ScoreL1, ArmElevatorState.ScoreL4),L12L4Trajectory);//Can't
    // use
    pathMap.put(key(ArmElevatorState.ScoreL1, ArmElevatorState.Initial), L12ITrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.StationIntake), L22STrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.ScoreL1), L22L1Trajectory);
    pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.ScoreL3), L22L3Trajectory);
    pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.Start), L22STTrajectory);
    // pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.ScoreL4),L22L4Trajectory);//Can't
    // use
    pathMap.put(key(ArmElevatorState.ScoreL2, ArmElevatorState.Initial), L22ITrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.StationIntake), L32STrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.ScoreL1), L32L1Trajectory);
    pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.ScoreL2), L32L2Trajectory);
    // pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.ScoreL4),L32L4Trajectory);//Can't
    // use
    pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.Initial), L32ITrajectory);
    pathMap.put(key(ArmElevatorState.Start, ArmElevatorState.ScoreL3), ST2L3Trajectory);
    pathMap.put(key(ArmElevatorState.Start, ArmElevatorState.ScoreL2), ST2L2Trajectory);
    pathMap.put(key(ArmElevatorState.Start, ArmElevatorState.StationIntake), ST2STrajectory);
    pathMap.put(key(ArmElevatorState.ScoreL3, ArmElevatorState.Start), L32STTrajectory);

    currentArmState = ArmElevatorState.Start;
    previousArmState = ArmElevatorState.Start;
  }

  private String key(ArmElevatorState previousState, ArmElevatorState currentState) {
    return previousState.name() + "->" + currentState.name();
  }

  public void setArmState(ArmElevatorState newState) {
    setArmState(currentArmState, newState);
  }

  public void setArmState(ArmElevatorState oldState, ArmElevatorState newState) {
    previousArmState = oldState;
    currentArmState = newState;

    Logger.recordOutput("previousStatus:", previousArmState.toString());
    Logger.recordOutput("currentStatus:", currentArmState.toString());

    currentPath = getPath(previousArmState, currentArmState);
    if (currentPath != null) {
      if (currentArmState != previousArmState) {
        trajStartTime = Timer.getFPGATimestamp();
      }
      runPath(currentPath); // take path
    } else {
      System.out.println("take defaylt behavior");
      // runMotionMagic(newState);; // take defaylt behavior
    }
  }

  private Trajectory2d getPath(ArmElevatorState previousState, ArmElevatorState currentState) {
    String key = key(previousState, currentState);
    return pathMap.getOrDefault(key, null); // if not find, return null
  }

  private void runPath(Trajectory2d path) {
    Setpoint2d setpoint = path.getSetpoint(Timer.getFPGATimestamp() - trajStartTime);
    arm.runGoal(setpoint.x1, setpoint.v1, 0.0, getArmTorque4Motor(setpoint.a1, setpoint.a0));
    elevator.runGoal(setpoint.x0, setpoint.v0, 0.0, getElevatorTorque(setpoint.a1, setpoint.a0));

    Logger.recordOutput("Arm/setpoint", setpoint.x1);
    Logger.recordOutput("Arm/setpointV", setpoint.v1);
    Logger.recordOutput("Arm/setpointA", setpoint.a1);
    Logger.recordOutput("Elevator/setpoint", setpoint.x0);
    Logger.recordOutput("Elevator/setpointV", setpoint.v0);
    Logger.recordOutput("Elevator/setpointA", setpoint.a0);
    Logger.recordOutput("timeEooer", Timer.getFPGATimestamp() - trajStartTime);
  }

  public int getLevel() {
    addHeight = driver.getRawButtonPressed(4);
    subHeight = driver.getRawButtonPressed(2);
    if (addHeight) {
      if (level < MAX_VALUE) {
        level++;
      }
    }

    if (subHeight) {
      if (level > MIN_VALUE) {
        level--;
      }
    }
    Logger.recordOutput("level", level);
    return level;
  }

  public double getArmTheta() {
    return arm.getAngleRads() + Math.toRadians(ArmConstants.armToThetaCompensation);
  }

  public double getArmTorque(double arm_a, double elevator_a) {
    double l = elevator.getPositionMeters();
    double r =
        Math.hypot(
            ElevatorConstants.elevatorMinimumQuantityCenterPositin
                * Math.sin(Math.toRadians(ArmConstants.armToThetaCompensation)),
            ElevatorConstants.elevatorMinimumQuantityCenterPositin
                    * Math.cos(Math.toRadians(ArmConstants.armToThetaCompensation))
                + l
                    / ElevatorConstants.elevatorMaximumPosition
                    * ElevatorConstants.elevatorMaximumQuantityCenterPositin);
    double I = armMass * r * r * 4 / 3; // * Math.cos(getArmTheta()) * Math.cos(getArmTheta());
    double dIdtheta = 0; // - 1/3*armMass * r * r * Math.sin(getArmTheta()*2);
    double TGravity = armMass * g * r * Math.cos(getArmTheta()) + dIdtheta * arm_a;
    double Tinertia = I * elevator_a + dIdtheta * arm.getOmegaRadPerSec() * arm.getOmegaRadPerSec();
    if (Math.abs(Tinertia) > Math.abs(TGravity) / 5) { // I>G/5, 5I/G>1
      Tinertia = Tinertia / 5.0 / Math.abs(Tinertia) * Math.abs(TGravity); // i new =  i / (5I/G)
    }
    double armTorqueCurrent =
        (TGravity + Tinertia) / ArmConstants.motorKT / ArmConstants.torqueRatio;
    return armTorqueCurrent;
  }

  public double getElevatorTorque(double arm_a, double elevator_a) {
    double r = elevator.getPositionMeters() + 0.7;
    double r_c = r / 2;
    double r_ea = 0.18;
    double fg = armMass * g * Math.sin(getArmTheta());
    double fe = armMass * elevator_a;
    double fc = armMass * arm.getOmegaRadPerSec() * arm.getOmegaRadPerSec() * r_c;
    double fa = armMass * r_ea * arm_a;
    double F = fg + fe - fc + fa;
    double T = F * ElevatorConstants.forceAjust;
    return T;
  }

  public double getArmTorque4Motor(double arm_a, double elevator_a) {
    return getArmTorque(arm_a, elevator_a) / 4;
  }

  @Override
  public void periodic() {
    switch (currentArmState) {
      case Stop -> {
        arm.stop();
        elevator.stop();
      }
      case Start -> {
        arm.stop();
        elevator.stop();
      }
      case ScoreL1 -> {
        if (currentPath != null) {
          runPath(currentPath);
        }
      }
      case ScoreL2 -> {
        if (currentPath != null) {
          runPath(currentPath);
        }
      }
      case ScoreL3 -> {
        if (currentPath != null) {
          runPath(currentPath);
        }
      }
    }

    Logger.recordOutput("currentArmState", currentArmState.toString());
    Logger.recordOutput("Arm/atGoarl", arm.atGoal());
    Logger.recordOutput("Arm/angleRads", arm.getAngleRads());
    Logger.recordOutput("Elevator/positionMeters", elevator.getPositionMeters());
  }
}
