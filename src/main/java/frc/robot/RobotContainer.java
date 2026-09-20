package frc.robot;

import java.security.Principal;
import java.sql.ClientInfoStatus;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Climbing;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.fieldConstants;
// import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Carriage.CarriageSystem;
import frc.robot.subsystems.Carriage.Elevator;
import frc.robot.subsystems.Carriage.Feeder;
import frc.robot.subsystems.Carriage.CarriageSystem.carriageState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Chassis.Gyro;
import frc.robot.subsystems.Chassis.DriveSubsystem.DriveMode;
import frc.robot.subsystems.Climber.ClimbingSubsystem;
import frc.robot.subsystems.Climber.ClimbingSubsystem.climbingState;
import frc.robot.subsystems.Intake.IntakeSystem;
import frc.robot.subsystems.Intake.IntakeSystem.intakeState;
import frc.robot.subsystems.Shooter.Flywheels;
import frc.robot.subsystems.Shooter.ShooterSystem;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.ShooterSystem.shooterState;
import frc.robot.subsystems.Vision.objectTracker;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Shooter.Triggershooting;

public class RobotContainer {
    /* Controllers */
    private final PS5Controller driver = new PS5Controller(0);
    private final PS5Controller operator = new PS5Controller(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroHeading = new JoystickButton(driver, 13);
    private final JoystickButton speaker = new JoystickButton(driver,8);
    private final JoystickButton amp = new JoystickButton(driver,2);
    private final JoystickButton pass = new JoystickButton(driver,6);
    private final JoystickButton groundIntake = new JoystickButton(driver,7);
    private final JoystickButton spit = new JoystickButton(driver, 1);
    private final JoystickButton eject = new JoystickButton(driver, 4);
    private final JoystickButton orinSwitch = new JoystickButton(driver, 10);
    private final JoystickButton shakehead = new JoystickButton(driver, 9);
    

    /*Operator*/
    private final POVButton climbingUP_Test = new POVButton(operator, 270);
    private final POVButton climbingDown_Test = new POVButton(operator, 90);
    private final POVButton climbingUP = new POVButton(operator, 0);
    private final POVButton climbingDown = new POVButton(operator, 180);
    private final JoystickButton toAmp = new JoystickButton(operator, 6);
    private final JoystickButton TrapScoring = new JoystickButton(operator, 7); 
    private final JoystickButton teleop_scoring = new JoystickButton(operator, 4); 
    private final JoystickButton sourceIntake = new JoystickButton(operator,5);

   

    /* Subsystems */
    private final LEDSubsystem LED=new LEDSubsystem();    
    private final Gyro gyro = new Gyro();
    private final VisionIO visionIO = new VisionIO();
    private final DriveSubsystem s_Swerve = new DriveSubsystem(gyro,visionIO,LED);
    private final objectTracker objectTracker = new objectTracker(visionIO,s_Swerve);
    private final Feeder feeder=new Feeder(); 
    private final Elevator elevator= new Elevator(); 
    private final ClimbingSubsystem climb = new ClimbingSubsystem();
    private final Flywheels flywheels = new Flywheels();
    private final Turret turret=new Turret();
    private final Triggershooting trigger=new Triggershooting(); 
    private final CarriageSystem carriage = new CarriageSystem(feeder, elevator, LED,climb,turret,trigger);  
    private final IntakeSystem intake=new IntakeSystem(LED,carriage,trigger,turret);
    private final ShooterSystem shooter = new ShooterSystem(s_Swerve,flywheels,turret,trigger,LED,intake,carriage);

    private final BlackholePilot blackholePilot = new BlackholePilot(visionIO, s_Swerve, objectTracker,feeder);

    private final List<String> autonomousModes=List.of("1","1+3","1+3+2left","F1+2");

    private SendableChooser<Command> chooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            s_Swerve
                .run(
                    () ->
                        s_Swerve.updateControllerInput(
                            // -driver.getRawAxis(translationAxis),
                            // -driver.getRawAxis(strafeAxis),
                            -driver.getRawAxis(translationAxis)/3,
                            -driver.getRawAxis(strafeAxis)/3,
                            -driver.getRawAxis(rotationAxis)
                            ))
                .withName("Drive Teleop Input")
            );

        // Configure the button bindings
        configureButtonBindings();

        // NamedCommands.registerCommand("intake",new InstantCommand(() -> {                        
        //     intake.setState(intakeState.INTAKE);
        // }));
        // NamedCommands.registerCommand("fastshoot",new InstantCommand(() -> {                        
        //     shooter.setState(shooterState.FASTSHOOT);
        // }));
        // NamedCommands.registerCommand("shoot",new InstantCommand(() -> {                        
        //     shooter.setState(shooterState.AUTONOMOUSPREIOD);
        // }));
        // NamedCommands.registerCommand("prepare",new InstantCommand(() -> {                        
        //     shooter.setState(shooterState.PREPARING);
        // }));
        // NamedCommands.registerCommand("autopoint",new InstantCommand(() -> {                        
        //     shooter.setState(shooterState.AUTOPOINT);
        // }));


        // chooser=new SendableChooser<>();
        // for (String mode : autonomousModes){
        //     chooser.addOption(mode, s_Swerve.autoCommand(mode));
        // }
        // SmartDashboard.putData("Auto Selector", chooser);
    }

    public void teleopInit(){
        s_Swerve.setState(DriveMode.TELEOP);
        intake.setState(intakeState.STOP);
        carriage.setState(carriageState.STOP);
        shooter.setState(shooterState.STOP);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        orinSwitch.onTrue(new InstantCommand(()->shooter.orinSwitch()));
        
        speaker.onTrue(new InstantCommand(() -> {
            // if(shooter.getorinSwitchstate()){
            //     s_Swerve.setState(DriveMode.AIMING);
            //     shooter.setState(shooterState.AUTO);
            // }else{
            //     shooter.setState(shooterState.DEFAULTSHOOT);
            // }
            shooter.setState(shooterState.Turret);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            s_Swerve.setState(DriveMode.TELEOP);
            // shooter.setState(shooterState.DEFAULT);
            shooter.setState(shooterState.Turret);
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
            trigger.stopTrigger();

        }));

        amp.onTrue(new InstantCommand(() -> {
            // carriage.setState(carriageState.AMPSCORING);
            // shooter.setState(shooterState.DEFAULT);
            // intake.setState(intakeState.SHOOTING);
            shooter.setState(shooterState.DEFAULT);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
            trigger.stopTrigger();
            shooter.setState(shooterState.STOP);
        }));

        shakehead.onTrue(new InstantCommand(() -> {
            shooter.setState(shooterState.TurretLowPosition);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            shooter.setState(shooterState.STOP);
        }));

        pass.onTrue(new InstantCommand(() -> {
            s_Swerve.setState(DriveMode.AIMING);
            shooter.setState(shooterState.PASSING);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            s_Swerve.setState(DriveMode.TELEOP);
            // shooter.setState(shooterState.DEFAULT);
            shooter.setState(shooterState.STOP);
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
        }));

        groundIntake.onTrue(new InstantCommand(() -> {
            shooter.setState(shooterState.DEFAULT);
            intake.setState(intakeState.INTAKE);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
            trigger.stopTrigger();
            // shooter.setState(shooterState.DEFAULT);
            shooter.setState(shooterState.STOP);
        }));

        eject.onTrue(new InstantCommand(() -> {
            intake.setState(intakeState.SPIT);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
            trigger.stopTrigger();
        }));

        spit.onTrue(new InstantCommand(() -> {
            shooter.setState(shooterState.SPIT);
        })).toggleOnFalse(new InstantCommand(() -> {                        
            // shooter.setState(shooterState.DEFAULT);
            shooter.setState(shooterState.STOP);
            intake.setState(intakeState.STOP);
            carriage.setState(carriageState.STOP);
        }));


        /* Opeartor Buttons */
        climbingUP.onTrue(new InstantCommand(() -> {
            climb.setState(climbingState.UP);
        })).toggleOnFalse(new InstantCommand(() -> {     
            climb.setState(climbingState.UP);
        }));

        climbingDown.onTrue(new InstantCommand(() -> {
            climb.setState(climbingState.DOWM);
        })).toggleOnFalse(new InstantCommand(() -> {     
            climb.setState(climbingState.DOWM);
        }));

        climbingUP_Test.onTrue(new InstantCommand(() -> {
            climb.setState(climbingState.FORWARD);
        })).toggleOnFalse(new InstantCommand(() -> {     
            climb.setState(climbingState.STOP);
        }));

        climbingDown_Test.onTrue(new InstantCommand(() -> {
            climb.setState(climbingState.BACKWARD);
        })).toggleOnFalse(new InstantCommand(() -> {     
            climb.setState(climbingState.STOP);
        }));

        //多个结构联动
        toAmp.onTrue(new InstantCommand(() -> {
            shooter.setState(shooterState.DEFAULT);
            carriage.setState(carriageState.TO_AMP);
        })).toggleOnFalse(new InstantCommand(() -> {     
            carriage.setState(carriageState.STOP);
            trigger.stopTrigger();
        }));

        TrapScoring.onTrue(new InstantCommand(() -> {
            carriage.setState(carriageState.TRAPSCORING);
        })).toggleOnFalse(new InstantCommand(() -> {     
            carriage.setState(carriageState.TRAPSCORING);
        }));;

        teleop_scoring.onTrue(new InstantCommand(() -> {
            carriage.setState(carriageState.TELEOP_ELEVATOR);
        })).toggleOnFalse(new InstantCommand(() -> {     
            carriage.setState(carriageState.STOP);
        }));;

        sourceIntake.onTrue(new InstantCommand(() -> {
            shooter.setState(shooterState.Source);
        })).toggleOnFalse(new InstantCommand(() -> {     
            // shooter.setState(shooterState.DEFAULT);
            shooter.setState(shooterState.STOP);
            carriage.setState(carriageState.STOP);
            intake.setState(intakeState.STOP);
        }));
        
    }

 
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
