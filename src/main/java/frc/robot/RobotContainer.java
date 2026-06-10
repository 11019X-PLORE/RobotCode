// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.UpperSructure;
import frc.robot.subsystems.UpperSructure.ArmElevatorState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOKrakenFOC;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKrakenFOC;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIOKrakenFOC;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private Arm arm;
  private Elevator elevator;
  private UpperSructure upperStructure;
  private final Intake intake;

  // Controller
  //   private final CommandXboxController controller = new CommandXboxController(0);
  private final PS4Controller controller = new PS4Controller(0);

  // button
  private final JoystickButton Lock0 = new JoystickButton(controller, 1);
  private final JoystickButton restgyro = new JoystickButton(controller, 13);
  private final JoystickButton stopX = new JoystickButton(controller, 9);
  private final JoystickButton robotangleControl = new JoystickButton(controller, 3);

  private final JoystickButton intakeCapture = new JoystickButton(controller, 7);
  private final JoystickButton score = new JoystickButton(controller, 8);
  private final JoystickButton coralLevel = new JoystickButton(controller, 5);
  private final JoystickButton stationIntake = new JoystickButton(controller, 6);
  private final POVButton addHeight = new POVButton(controller, 0);
  private final JoystickButton up = new JoystickButton(controller, 4);
  private final JoystickButton dowm = new JoystickButton(controller, 2);

  // private final POVButton qf = new POVButton(controller, 0); // right
  // private final POVButton qb = new POVButton(controller, 180); // left
  // private final POVButton df = new POVButton(controller, 90);
  // private final POVButton db = new POVButton(controller, 270);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        arm = new Arm(new ArmIOKrakenFOC());
        elevator = new Elevator(new ElevatorIOKrakenFOC());
        upperStructure = new UpperSructure(arm, elevator, controller);
        intake = new Intake(new IntakeIOKrakenFOC());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        upperStructure = new UpperSructure(arm, elevator, controller);
        intake = new Intake(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        upperStructure = new UpperSructure(arm, elevator, controller);
        intake = new Intake(new IntakeIOSim());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.SysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.SysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.SysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.SysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    Lock0.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    stopX.onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    restgyro.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .x()
    robotangleControl.whileTrue(
        Commands.startRun(
            () -> {
              aimController.reset();
            },
            () -> {
              drive.runVelocity(
                  new ChassisSpeeds(
                      0.0, 0.0, aimController.calculate(vision.getTargetX(0).getRadians())));
            },
            drive)); // TODO: 机器角度控制 需要在某个地方重写 可以参考10711

    coralLevel.onTrue(
        new InstantCommand(
            () -> {
              ArmElevatorState currentState;
              switch (upperStructure.getLevel()) {
                case 1:
                  currentState = ArmElevatorState.ScoreL1;
                  break;
                case 2:
                  currentState = ArmElevatorState.ScoreL2;
                  break;
                case 3:
                  currentState = ArmElevatorState.ScoreL3;
                  break;
                default:
                  currentState = ArmElevatorState.Initial;
                  break;
              }
              upperStructure.setArmState(currentState);
            }));

    intakeCapture
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setState(IntakeState.FORWAR);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  intake.setState(IntakeState.STOP);
                }));

    score
        .onTrue(
            new InstantCommand(
                () -> {
                  intake.setState(IntakeState.BACKWARD);
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  intake.setState(IntakeState.STOP);
                }));

    stationIntake.onTrue(
        new InstantCommand(
            () -> {
              upperStructure.setArmState(ArmElevatorState.StationIntake);
            }));

    up.onTrue(
        new InstantCommand(
            () -> {
              arm.runArmPosition(1.2, 0.0, 0.0, 0.0);
            }));

    dowm.onTrue(
        new InstantCommand(
            () -> {
              arm.runArmPosition(0.5, 0.0, 0.0, 0.0);
            }));

    // qf.whileTrue(drive.SysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // qb.whileTrue(drive.SysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // df.whileTrue(drive.SysIdDynamic(SysIdRoutine.Direction.kForward));
    // db.whileTrue(drive.SysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void initTeleop() {
    upperStructure.setArmState(ArmElevatorState.Start); // need to be changed to Start
  }

  public void disableContainer() {
    upperStructure.setArmState(ArmElevatorState.Stop);
  }
}
