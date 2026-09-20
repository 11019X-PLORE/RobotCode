package frc.robot.subsystems.Chassis;

// import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import javax.swing.text.html.parser.Element;

import org.opencv.core.Mat;

// import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Chassis.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Chassis.controllers.AimController;
import frc.robot.subsystems.Chassis.controllers.AutoAlignController;
import frc.robot.subsystems.Chassis.controllers.TeleopDriveController;
import frc.robot.subsystems.Intake.IntakeSystem.intakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import choreo.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// import choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;


public class DriveSubsystem extends SubsystemBase {

    PoseEstimator m_poseEstimator;
    VisionIO m_visionIO;
    Gyro m_gyro;
    LEDSubsystem m_LED;

    public SwerveModule[] mSwerveMods;

    public Rotation2d directionFromSpeaker;
    public boolean speakerTagDetected=false;
    private Field2d m_field = new Field2d();
    


    private Notifier odometryNotifier;

    private final TeleopDriveController teleopDriveController;
    private final AimController aimingController;
    private final AutoAlignController autoAlignController;

    private ChassisSpeeds desireSpeeds=new ChassisSpeeds(0,0,0);
    private ChassisSpeeds currentDesireSpeeds=new ChassisSpeeds(0,0,0);

    private Translation2d lastVelocity=new Translation2d(0,0);

    public enum DriveMode{
        STOP,
        TELEOP,
        AUTONOMOUS,
        AUTO_ALIGN,
        AIMING
    }
    DriveMode currentDriveMode=DriveMode.STOP;
    public DriveSubsystem(Gyro gyro,VisionIO visionIO,LEDSubsystem LED) {
        this.m_visionIO=visionIO;
        this.m_LED=LED;
        this.m_gyro=gyro;
        this.m_gyro.zeroHeading();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        m_poseEstimator=new PoseEstimator(new Pose2d(0,0,new Rotation2d(0)),0.01,Constants.Swerve.swerveKinematics);
        odometryNotifier = new Notifier(this::updateOdometry);
        startOdometry();
        teleopDriveController = new TeleopDriveController(m_poseEstimator);
        aimingController= new AimController(m_poseEstimator);
        autoAlignController=new AutoAlignController(m_poseEstimator);
        SmartDashboard.putData("Field",m_field);
    }

    public void updateControllerInput(
      double controllerX, double controllerY, double controllerOmega) {
        if (DriverStation.isTeleopEnabled()) {
        teleopDriveController.updateControllerInput(
            controllerX, controllerY, controllerOmega);
        }
    }
    public void setAimTarget(Rotation2d desireRotation2d,double desireAngularSpeed){
        aimingController.setGoal(desireRotation2d,desireAngularSpeed);
    }
    public void setAimTarget(Rotation2d desireRotation2d,double desireAngularSpeed,double tolerance){
        aimingController.setGoal(desireRotation2d,desireAngularSpeed,tolerance);
    }
    public boolean atAimTarget(){
        return aimingController.atGaol();
    }
    public double getAimError(){
        return aimingController.getError();
    }
    public void setAlignTarget(Pose2d desirePose2d){
        autoAlignController.setGoal(desirePose2d);
    }
    public void setAlignTarget(Pose2d desirePose2d,DriverStation.Alliance allianceColor){
        autoAlignController.setGoal(desirePose2d,allianceColor);
    }
    
    public boolean atSetpoint(){
        return autoAlignController.atGaol();
    }

    private void updateOdometry() {
        // double startTime=Timer.getFPGATimestamp();
        // double processingNoise=Constants.Swerve.odometryProcessingNoisePerMeter;
        // if(!DriverStation.isAutonomous()&&!DriverStation.isAutonomousEnabled()){
        //     processingNoise=0.001;//TODO: if not working, delete this line
        // }
        // m_poseEstimator.updateDrive(getGyroYaw(), getModulePositions(),processingNoise);
        // swerveOdometry.update(getGyroYaw(), getModulePositions());
        m_poseEstimator.updateDrive(getGyroYaw(), getModulePositions(),Constants.Swerve.odometryProcessingNoisePerMeter);
        Translation2d totalAcceleration=m_gyro.getTotalAcceleration().toTranslation2d().rotateBy(m_poseEstimator.getEstimatedPosition().getRotation());
        ChassisSpeeds currentChassisSpeeds=getFieldRelativeSpeeds();
        Translation2d currentVelocity=new Translation2d(currentChassisSpeeds.vxMetersPerSecond,currentChassisSpeeds.vyMetersPerSecond);
        Translation2d estimatedAcceleration=currentVelocity.minus(lastVelocity).div(Constants.Swerve.odometryPeriod);
        // m_poseEstimator.addErrorInEstimate(totalAcceleration.getDistance(estimatedAcceleration)*Math.pow(Constants.Swerve.odometryPeriod,2));
        // SmartDashboard.putNumber("acceleration difference", totalAcceleration.getDistance(estimatedAcceleration));
        // SmartDashboard.putNumber("odometry periode", Timer.getFPGATimestamp()-startTime);
    }
    public void stopOdometry() {
        if (odometryNotifier != null) {
            odometryNotifier.stop();
        }
    }

    public void startOdometry() {
        if (odometryNotifier != null) {
            odometryNotifier.startPeriodic(Constants.Swerve.odometryPeriod);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            //SmartDashboard.putString(mod.moduleNumber+" Desire",desiredStates[mod.moduleNumber].toString());
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return swerveOdometry.getPoseMeters();
        return m_poseEstimator.getEstimatedPosition();
    }
    public Pose2d getPoseWithTime(double timestamp) {
        // return swerveOdometry.getPoseMeters();
        if(timestamp<Timer.getFPGATimestamp()){
            return m_poseEstimator.getRobotPoseWithTime(timestamp);
        }else{
            ChassisSpeeds currentChassisSpeeds=getFieldRelativeSpeeds();
            return m_poseEstimator.getEstimatedPosition().plus( new Transform2d(
                currentChassisSpeeds.vxMetersPerSecond,
                currentChassisSpeeds.vyMetersPerSecond,
                new Rotation2d(currentChassisSpeeds.omegaRadiansPerSecond)
            ).times(timestamp-Timer.getFPGATimestamp()));
        }
    }

    public void setPose(Pose2d pose) {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.setPose(pose);
        setHeading(pose.getRotation());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_poseEstimator.setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red){
                m_poseEstimator.setRotation(new Rotation2d(Math.PI));
            }else{
                m_poseEstimator.setRotation(new Rotation2d(0));
            }
        }
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public Rotation2d getGyroYaw() {
        return m_gyro.getGyroYaw();
    }

    public Pose2d inversePose2dUsingAlliance(Pose2d pose,DriverStation.Alliance allianceColor){
        if(allianceColor!=getAlliance()){
            return new Pose2d(Constants.fieldConstants.fieldSize.getX()-pose.getX(),pose.getY(),new Rotation2d(Math.PI).minus(pose.getRotation()));
        }
        return pose;
    }
    public Translation2d inverseTranslationUsingAlliance(Translation2d translation,DriverStation.Alliance allianceColor){
        if(allianceColor!=getAlliance()){
            return new Translation2d(Constants.fieldConstants.fieldSize.getX()-translation.getX(),translation.getY());
        }
        return translation;
    }


    public ChassisSpeeds getFieldRelativeSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),getHeading());
    }

    public double getChassisSpeed(){
        return Math.sqrt(Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2)+Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2));
    }


    
    //return the chassis speed for the followpathholonomic method
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    //ChassisSpeeds supplier for the followpathholonomic method
    public void setChassisSpeeds (ChassisSpeeds speeds) {
        // SmartDashboard.putString("auto desire speeds", speeds.toString());
        // if (currentDriveMode==DriveMode.AIMING&&DriverStation.isAutonomous()&&DriverStation.isAutonomousEnabled()){
        //     desireSpeeds.omegaRadiansPerSecond=aimingController.getAngularSpeeds();
        // }
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxModuleSpeed);
        setModuleStates(moduleStates);
    }
    //auto***************************************************************************************************

    // // generate a trajectory(from .path to .json)
    // public PathPlannerPath generatePath(String pathName){
    //     return PathPlannerPath.fromPathFile(pathName);
    // }
    
    // public PathPlannerPath generateChoreoPath(String pathName){
    //     return PathPlannerPath.fromChoreoTrajectory(pathName);
    // }
    //return an auto command
    // public Command followPathCommand(PathPlannerPath path){
    //     // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
    //     return 
    //         new FollowPathHolonomic(
    //             path,
    //             this::getPose, // Robot pose supplier
    //             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                         new PIDConstants(Constants.AutoConstants.kPTranslationController, 0.0, 0.0), // Translation PID constants
    //                         new PIDConstants(Constants.AutoConstants.kPRotationController, 0.0, 0.0), // Rotation PID constants
    //                         Constants.Swerve.maxModuleSpeed, // Max module speed, in m/s
    //                         Math.sqrt(2)*Constants.Swerve.wheelBase/2, // Drive base radius in meters. Distance from robot center to furthest module.
    //                         new ReplanningConfig() // Default path replanning config. See the API for the options here
    //                 ),
    //                 () -> {
    //                     // Boolean supplier that controls when the path will be mirrored for the red alliance
    //                     // This will flip the path being followed to the red side of the field.
    //                     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //                     var alliance = DriverStation.getAlliance();
    //                     if (alliance.isPresent()) {
    //                         return alliance.get() == DriverStation.Alliance.Red;
    //                     }
    //                     return false;
    //                 },
    //                 this // Reference to this subsystem to set requirements
    //         );
    // }

    // public Command followPathCommand(PathPlannerPath path) {
    //     HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
    //         new PIDConstants(Constants.AutoConstants.kPTranslationController, 0.0, 0.0),
    //         new PIDConstants(Constants.AutoConstants.kPRotationController, 0.0, 0.0),
    //         Constants.Swerve.maxModuleSpeed,
    //         Math.sqrt(2) * Constants.Swerve.wheelBase / 2,
    //         new ReplanningConfig()
    //     ); 

    //     return AutoBuilder.followHolonomicPath(
    //         path,
    //         this::getPose,
    //         this::getRobotRelativeSpeeds,
    //         this::setChassisSpeeds,
    //         config,
    //         this::isRedAlliance,
    //         this
    //     );
    // }

    // public Command followPathCommand(ChoreoTrajectory path){
    // var thetaController = new PIDController(Constants.AutoConstants.kPRotationController, 0, 0);
    // var yController=new PIDController(Constants.AutoConstants.kPTranslationController, 0.0, 0.0);
    // var xController=new PIDController(Constants.AutoConstants.kPTranslationController, 0.0, 0.0);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Command swerveCommand = Choreo.choreoSwerveCommand(
    //     path, // Choreo trajectory from above
    //     this::getPose,  // A function that returns the current field-relative pose of the robot: your
    //                            // wheel or vision odometry
    //     yController, // PIDController for field-relative X
    //                                                                                // translation (input: X error in meters,
    //                                                                                // output: m/s).
    //     xController, // PIDController for field-relative Y
    //                                                                                // translation (input: Y error in meters,
    //                                                                                // output: m/s).
    //     thetaController, // PID constants to correct for rotation
    //                      // error
    //     (ChassisSpeeds speeds) -> setChassisSpeeds( // needs to be robot-relative
    //         DriverStation.getAlliance().get() == DriverStation.Alliance.Red?
    //         ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_poseEstimator.getEstimatedPosition().getRotation().plus(new Rotation2d(Math.PI))):
    //         ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_poseEstimator.getEstimatedPosition().getRotation())
    //         ),
    //     () -> {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //     }, 
    //     this // The subsystem(s) to require, typically your drive subsystem only
        
    // );

    // return Commands.sequence(
    //     swerveCommand,
    //     new InstantCommand(()->
    //     setChassisSpeeds(new ChassisSpeeds(0,0,0))
    //     ));
    // }
    // public SequentialCommandGroup autoCommand(String pathName){
    //     // ChoreoTrajectory path = Choreo.getTrajectory(pathName);
    //     PathPlannerPath path=generateChoreoPath(pathName);
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             setState(DriveMode.AUTONOMOUS);
    //             setPose(inversePose2dUsingAlliance(path.getPreviewStartingHolonomicPose(),DriverStation.Alliance.Blue));
                
    //         }
    //         ),
    //         followPathCommand(path)
    //     );}

    

    public DriverStation.Alliance getAlliance(){
        try{
            return DriverStation.getAlliance().get();
        }catch(Exception e){
            return DriverStation.Alliance.Red;
        }
    }
    
    public void setState(DriveMode state){
        this.currentDriveMode=state;
        // SmartDashboard.putString("state",state.toString());
    }

    public ChassisSpeeds getDriverDesireSpeeds(){
        return desireSpeeds;
    }
    
    public Translation2d optimizeDesireChassisVelocity(Translation2d desireChassisVelocity,Translation2d currentDesireChassisVelocity){
        Translation2d currentChassisSpeed=currentDesireChassisVelocity;
        Rotation2d headingAngle=currentChassisSpeed.getAngle();
        Rotation2d desireDirection=desireChassisVelocity.getAngle();
        double rotatedCurrentSpeed=currentChassisSpeed.rotateBy(desireDirection.times(-1)).getX();
        double desireChassisSpeed=desireChassisVelocity.getNorm();

        Translation2d deltaV=desireChassisVelocity.minus(currentChassisSpeed);
        Translation2d acceleration=new Translation2d(Constants.Swerve.maxCentripetalAcceleration,0).rotateBy(deltaV.getAngle());
        if (rotatedCurrentSpeed<0){
        desireChassisSpeed=0;
        }else if(rotatedCurrentSpeed<desireChassisSpeed){
            desireChassisSpeed=rotatedCurrentSpeed;
        }
        desireChassisVelocity=new Translation2d(desireChassisSpeed,0).rotateBy(desireDirection); 
        Translation2d rotatedDesireAcceleration=acceleration.rotateBy(headingAngle.times(-1));
        if((Constants.Swerve.maxAcceleration-rotatedDesireAcceleration.getX())<0||currentChassisSpeed.getNorm()==0){
            acceleration=new Translation2d(Constants.Swerve.maxAcceleration,0).rotateBy(acceleration.getAngle());
        }
        if((Constants.Swerve.maxDeceleration+rotatedDesireAcceleration.getX())<0){
            acceleration=new Translation2d(Constants.Swerve.maxDeceleration,0).rotateBy(acceleration.getAngle());
        }
        Translation2d changeSpeed=acceleration.times(Constants.Swerve.loopDuration);
            if(changeSpeed.getNorm()<deltaV.getNorm()){
                desireChassisVelocity=currentChassisSpeed.plus(changeSpeed);
        }
        return desireChassisVelocity;
    }

    public double optimizeDesireChassisAngularSpeed(double desireAngularSpeed,double currentDesireChassisAngularSpeed){
        double currentAngularSpeed = currentDesireChassisAngularSpeed;
        double angularAcceleration;
        double deltaOmega=desireAngularSpeed-currentAngularSpeed;
        if (Math.abs(desireAngularSpeed)>Math.abs(currentAngularSpeed)){
            angularAcceleration=Constants.Swerve.maxAngularAcceleration;
        }else{
            angularAcceleration=Constants.Swerve.maxAngularDeceleration;
        }
        double changeSpeed=angularAcceleration*Constants.Swerve.loopDuration;
        if(deltaOmega<0){
            changeSpeed=changeSpeed*-1;
        }
        if(Math.abs(changeSpeed)<Math.abs(deltaOmega)){
            desireAngularSpeed=currentAngularSpeed+changeSpeed;
        }
        return desireAngularSpeed;
    }
    public ChassisSpeeds optimizeDesireChassisSpeeds(ChassisSpeeds desireChassisSpeeds, ChassisSpeeds currentDesireChassisSpeeds){
        Translation2d desireVelocity=new Translation2d(desireChassisSpeeds.vxMetersPerSecond,desireChassisSpeeds.vyMetersPerSecond);
        double rotation=desireChassisSpeeds.omegaRadiansPerSecond;
        Translation2d currentDesireSpeed=new Translation2d(currentDesireChassisSpeeds.vxMetersPerSecond,currentDesireChassisSpeeds.vyMetersPerSecond);
        double currentDesireAngularSpeed=currentDesireChassisSpeeds.omegaRadiansPerSecond;
        desireVelocity=optimizeDesireChassisVelocity(desireVelocity,currentDesireSpeed);
        currentDesireSpeed=desireVelocity;
        rotation=optimizeDesireChassisAngularSpeed(rotation,currentDesireAngularSpeed);
        currentDesireAngularSpeed=rotation;
        return new ChassisSpeeds(
            desireVelocity.getX(), 
            desireVelocity.getY(), 
            rotation);
    }
    public ChassisSpeeds limitDesireSpeeds(ChassisSpeeds desireSpeeds,ChassisSpeeds acturalSpeeds){
        Translation2d desireVelocity=new Translation2d(desireSpeeds.vxMetersPerSecond,desireSpeeds.vyMetersPerSecond);
        Translation2d acturalVelocity=new Translation2d(acturalSpeeds.vxMetersPerSecond,acturalSpeeds.vyMetersPerSecond);
        Translation2d limitedVelocity;
        if(Math.abs(desireVelocity.getDistance(acturalVelocity))>Constants.Swerve.velocityControlTolerance){
            limitedVelocity=acturalVelocity;
        }else{
            limitedVelocity=desireVelocity;
        }
        if(Math.abs(desireSpeeds.omegaRadiansPerSecond-acturalSpeeds.omegaRadiansPerSecond)>Constants.Swerve.angularVelocityControlTolerance){
            return new ChassisSpeeds(limitedVelocity.getX(),limitedVelocity.getY(),acturalSpeeds.omegaRadiansPerSecond);
        }else{
            return new ChassisSpeeds(limitedVelocity.getX(),limitedVelocity.getY(),desireSpeeds.omegaRadiansPerSecond);
        }
    }
    @Override
    public void periodic(){      
        // SmartDashboard.putNumber("getGyro", getGyroYaw().getDegrees());
        desireSpeeds = limitDesireSpeeds(desireSpeeds, getFieldRelativeSpeeds());
        ChassisSpeeds teleopSpeeds = teleopDriveController.getDesireSpeeds();
        switch (currentDriveMode) {
            case TELEOP -> {
                // Plain teleop drive
                desireSpeeds = teleopSpeeds;
                // desireSpeeds=optimizeDesireChassisSpeeds(desireSpeeds, currentDesireSpeeds);
                setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(desireSpeeds, m_poseEstimator.getEstimatedPosition().getRotation()));
            }
            case AUTO_ALIGN -> {
                desireSpeeds=autoAlignController.getDesireSpeeds();
                desireSpeeds=optimizeDesireChassisSpeeds(desireSpeeds, currentDesireSpeeds);
                setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(desireSpeeds, m_poseEstimator.getEstimatedPosition().getRotation()));
                m_LED.update(LEDState.AUTO);
            }
            case AIMING->{
                desireSpeeds=teleopSpeeds;
                desireSpeeds.omegaRadiansPerSecond=aimingController.getAngularSpeeds();
                desireSpeeds=optimizeDesireChassisSpeeds(desireSpeeds, currentDesireSpeeds);
                setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(desireSpeeds, m_poseEstimator.getEstimatedPosition().getRotation()));
                m_LED.update(LEDState.AUTO);
            }
            case AUTONOMOUS->{

            }
            default ->{
                desireSpeeds = teleopSpeeds;
                desireSpeeds=optimizeDesireChassisSpeeds(desireSpeeds, currentDesireSpeeds);
                setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(desireSpeeds, m_poseEstimator.getEstimatedPosition().getRotation()));
            }
        }
        
        currentDesireSpeeds=desireSpeeds;

        
        List<TimestampedVisionUpdate> visionPoses=m_visionIO.getRobotPose3ds();
        // if(!DriverStation.isAutonomous()&&!DriverStation.isAutonomousEnabled()){
            m_poseEstimator.visionInput(visionPoses,m_gyro.getGyroYaw(),
                Constants.PoseEstimator.gyroTolerance,
                Constants.PoseEstimator.errorTolerance,
                Constants.PoseEstimator.heightTolerance);
        // }
        


        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        // SmartDashboard.putString("drivemode", currentDriveMode.toString());
        // SmartDashboard.putString("chassisSpeed", getFieldRelativeSpeeds().toString());
        // SmartDashboard.putString("desireSpeeds", desireSpeeds.toString());
        //print debuging information
        // for(SwerveModule mod : mSwerveMods){
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }
    }

}