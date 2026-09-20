package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.Intake.IntakeSystem.intakeState;

public final class Constants {
    public static final double stickDeadband = 0.5; //0.1
    public static enum drivers{Geoffrey,James}
    public static drivers driver = drivers.James;
    
    public static final class physicalConstants{
        
        public static enum locations{Shanghai,Beijing,Huston}

        public static final locations location=locations.Beijing;
        public static double g=switch(location){
            case Shanghai->-9.7940;
            case Beijing->-9.801;
            case Huston->-9.793;
			default -> -9.8;
        };
    }
    public static final class  fieldConstants {
        public static final Translation2d fieldSize=new Translation2d(16.54,8.21);

        public static final Translation3d SpeakerTranslation= new Translation3d(0.1, 5.548, 2);
        public static final double speakerWidth=1;
        public static final Translation2d passingTarget=new Translation2d(0.5,7.5);
        public static final double passingHeight=3.0;

        public static final Pose2d ampScoringPose=new Pose2d(1.8,7.65,new Rotation2d(Math.PI/2));
        
    }
    public static final class OIConstants {
        public static final int kFirstDriverControllerPort = 0;
        public static final int kSecondDriverControllerPort = 1;
        public static final int kTestControllerPort = 2;
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;//headless mode
        public static final double stickDeadband = 0.02;    
      }

    public static final class Swerve {
        public static final int pigeonID = 0;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(0.615/0.0254); 
        public static final double wheelBase = Units.inchesToMeters(0.540/0.0254); 
        public static final double wheelCircumference = 0.096*Math.PI;
        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.12;
        public static final double angleGearRatio = 18.0;//chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 50;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final int driveTorgueCurrent=40;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = 70.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKV = 3.0;

         /* Drive Motor PID Values */
         public static final double driveKP = 0.001; //11.9v:9.9277E-14;12.4v:7.8004E-17
         public static final double driveKI = 0.0; 
         public static final double driveKD = 0.0001;//0.0;
         public static final double driveKF = 0.0;//0.0;
 

           /* Drive Motor Characterization Values From SYSID */
           public static final double driveKS = 0.15444; //11.9v:0.15444;12.4v:0.19218
           public static final double driveKV = 2.33;  //11.9v:2.4459;12.4v:2.228
           public static final double driveKA = 0.15101; //11.9v:0.15101;12.4v:0.22843
         

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.1; //TODO: This must be tuned to specific robot
        public static final double maxModuleSpeed = 3.5; //TODO: This must be tuned to specific robot

        public static final double maxAcceleration = 4.5;
        public static final double maxDeceleration= 44.0;

        public static final double maxCentripetalAcceleration = 100.0;

        public static final double loopDuration = 0.02; //in second

        public static final double velocityControlTolerance=0.1;

        /** Radians per Second */
        public static final double stationaryAngularVelocity = 1.5*Math.PI*2; //TODO: This must be tuned to specific robot
        public static final double maxRadius=0.4;
        public static final double maxAngularAcceleration = 30.0;
        public static final double maxAngularDeceleration = 90.0;

        public static final double angularVelocityControlTolerance=0.2;
        

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double odometryPeriod=0.004;
        public static final double odometryProcessingNoisePerMeter=0.01;
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            // the bigger the offset, the smaller the setPosition, thus the relative CW point of the start angle
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-148.1835);//37.17
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.Clockwise_Positive,InvertedValue.Clockwise_Positive);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            // the bigger the offset, the smaller the setPosition, thus the relative CW point of the start angle
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(129.814);//-43.825
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.Clockwise_Positive);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            // the bigger the offset, the smaller the setPosition, thus the relative CW point of the start angle
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(124.804);//303.222-180
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.Clockwise_Positive,InvertedValue.Clockwise_Positive);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            // the bigger the offset, the smaller the setPosition, thus the relative CW point of the start angle
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(144.052);//-29.10
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset,
                InvertedValue.CounterClockwise_Positive,InvertedValue.Clockwise_Positive);
        }
 
    }


    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPTranslationController = 4;
        public static final double kPRotationController = 3;

        public static final double kPAligningController=7;
        public static final double kDAligningController=0.8;
        public static final double kPAimingController=4.0;
        public static final double kDAimingController=0.0;

        public static final double aimTolerance=Math.toRadians(5);
        public static final double translationTolerance=0.05;

        
    }

    public static final class BlackholeVisionConstants{
        public static final String[] tableNames={"bottom","top"};
        public static final String config="{\"time\":10,\"cameras\": ["+
            "{\"name\": \"top\",\"ID\": \"/dev/camera_top_left\",\"pose\":{\"x\":0.1284,\"y\":0,\"z\":0.6666,\"roll\":0,\"pitch\":0,\"yaw\":3.141592653589793},\"cameraMatrix\": [[906.11130401,0,691.15552533],[0,906.35171032,371.55847183],[0,0,1]],\"distortionCoeffs\": [0.02281145,-0.01416523,0.00050521,0.0011957,-0.02318725]},"+
            "{\"name\": \"bottom\",\"ID\": \"/dev/camera_bottom_left\",\"pose\":{\"x\":0.37464,\"y\":0,\"z\":0.3988,\"roll\":0,\"pitch\":0,\"yaw\":0},\"cameraMatrix\": [[906.11130401,0,691.15552533],[0,906.35171032,371.55847183],[0,0,1]],\"distortionCoeffs\": [0.02281145,-0.01416523,0.00050521,0.0011957,-0.02318725]}"+
            "],\"capture\": {\"resolution\": {\"width\": 1280,\"height\":800},\"FPS\": 100,\"autoExposure\": 1,\"exposure\": 15,\"gain\": 30,\"brightness\": 30,\"contrast\": 100}"+
            ",\"apriltagDetector\": {\"resolution\": {\"width\": 1600,\"height\":1200},\"aruco\": \"DICT_APRILTAG_36H11\",\"rejectingIds\":[]}"+
            ",\"tagPoseEstimator\": {\"tagSize\":0.165}"+
            ",\"Calibration\":{\"charuco_dict\": \"DICT_6X6_250\",\"charuco_board\": {\"squares_x\": 5,\"squares_y\": 7,\"square_length\": 0.037,\"marker_length\": 0.022},\"image_path\": \"/home/jetson/Desktop/BlackHoleVision/BlackHoleVision1.0(5,19,2024)/calibration/calibration_videos/video_*.jpg\"}"+
            ",\"objectDetection\":{\"modelName\":\"last.pt\",\"thresholdDistance\":0.5 },\"output\":{\"networkTable\":{\"teamNumber\":1001}}}\r\n";
        public static final double confiTolerance=0.1;
        public static final double communicationLatency=0.1;
    }

    public static final class PoseEstimator {
        public static final double errorTolerance=1.5; 
        public static final double gyroTolerance=Math.toRadians(8);
        public static final double heightTolerance=0.5;
        public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class Shooter {
        public static final int turretMotorID = 17;
        public static final int bottomFlywheelLeftMotorID = 18;
        public static final int topFlywheelMotorID = 19;

        public static final double flywheelDiameter=0.106;

        public static final double flywheelGearRatio = 1.0;
        public static final double turretGearRatio = 66.0*180.0/(18.0*8.0);

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Drive Motor PID Values */
        public static final double flyWheelKP = 0.7; //TODO: This must be tuned to specific robot
        public static final double flyWheelKI = 0.0;
        public static final double flyWheelKD = 0;//0.0;
        public static final double turretKP = 80; 
        public static final double turretKI = 0.0;
        public static final double turretKD = 0.8;//0.0;
        public static final double turretKG = 0.6;

        /* Drive Motor Characterization Values From SYSID */
        public static final double flyWheelKS = 0.14185; //TODO: This must be tuned to specific robot
        public static final double flyWheelKV = 0.12;
        public static final double flyWheelKA = 0.0;
        public static final double turretKS = 0; 
        public static final double turretKV = 0;
        public static final double turretKA = 0;
        public static final double turretAcceleration=3.0;
        public static final double turretVelocity=1.0;

        /* Neutral Modes */
        public static final NeutralModeValue flywheelNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue turretNeutralMode = NeutralModeValue.Brake;
        
        public static final double flywheelTolerance = 2.0;
        public static final double turretTolerance=Math.toRadians(7);

        public static final Translation2d turretPivotTranslation=new Translation2d(0.3156,0.0694);
        public static final double turretRadius=0.494;

        public static final double preparingRange=20.0;
        public static final double defaultLowSpeed = 15.0;
        public static final double sourceSpeed = -10.0;
        public static final double defaultHighSpeed = 20.0;
        public static final double PassSpeed = 18.0;

        public static final double turretDefaultAngle=Math.toRadians(53);//radius
        public static final double turretAutoDefaultAngle=Math.toRadians(56);//radius
        public static final double TurretLowPosition=Math.toRadians(0);

        public static final double turretMinimumAngle=Math.toRadians(10);//radius
        public static final double turretMaximumAngle=Math.toRadians(60);//radius
        public static final double turretInitialAngle=Math.toRadians(-6);
        public static final double turretIntakingAngle=Math.toRadians(20);
        public static final double turretPassAngle=Math.toRadians(20);
        public static final double turretsourceAngle=Math.toRadians(20);

        public static final double turretAutoAngle=Math.toRadians(42.0);
        public static final double turretAutoAngleError=Math.toRadians(2.0);

        public static final double predictTime=0.02;
        public static final double errorDriveRadius=2.25;
        public static final double errorDrivePassRadius=7.0;

        /*Trriger Constants */
        public static final int triggerMotorID = 16;
        public static final int photoSensorChannel = 7;

        public static final double triggerGearRatio = 1.0; 
        public static final double triggerRollerDiameter=0.0508;
        
        public static final double trrigerKP = 0.1; 
        public static final double trrigerKI = 0.0; 
        public static final double trrigerKD = 0.0; 
        public static final double trrigerKS = 0.2; 
        public static final double trrigerKV = 0.12; 
        public static final double trrigerKA = 0.0; 

        public static final double FowardVel = 3.53;
        public static final double BackwardVel = -4.71;
        public static final double ShootingVel = 12.0;
        public static final double intakeVel = 0.8;
        public static final double ampScoreVel = -9.0;
        public static final double spitVel = -9.0;
    }
    public static final class Intake {
        public static final int intakeMotorID = 13;

        public static final int photoSensorChannel=9;
        /* Module Gear Ratios */
        public static final double intakeGearRatio = 1.0;

        public static final double intakeRollerDiameter=0.03;
        /* Motor Inverts */
        public static final boolean intakeMotorInvert = false;

        public static final int intakeCurrentLimit = 50;
        public static final int intakeCurrentThreshold = 60;
        public static final double intakeCurrentThresholdTime = 0.1;
        public static final boolean intakeEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double KP = 0.11; 
        public static final double KI = 0.0; 
        public static final double KD = 0.0001; 
        public static final double KS = 0.0; 
        public static final double KV = 0.12; 
        public static final double KA = 0.0; 

        /* Neutral Modes */
        public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake;
        public static final double intakeForwardSpeed = 0.1;
        public static final double intakeBackwardSpeed = 0.1;


        public static final double shootingVelocity=15.0;
        public static final double intakeVelocity=25.0;
        public static final double ejectingVelocity=-8.8;
        public static final double autoVelocity=25.0;
    }
    public static final class LEDConstants{
        public static final int LEDPort = 0;
        public static final int length = 30;

        //solidColor
        public static final int initializelength = 0;
        public static final double TwoColorlength = 0.1;

        //strobe
        public static final double strobeDuration = 0.3;
        public static final double EstoppedstrobeDuration = 0.3;
        public static final double PickstrobeDuration = 0.25;
        public static final double CaststrobeDuration = 0.25;
        public static final double ScorestrobeDuration = 0.2;

        //rainbow
        public static final double ClimbrainbowcycleLength = 10.0;
        public static final double ClimbrainbowDurtion = 1.0;

        //breath
        public static final double breathDuration = 0.3;
        public static final double EnablebreathDuration = 0.6;

        //wave
        public static final double waveExponent = 0.4;
        public static final double DefaultwaveLength = 5;
        public static final double DefaultwaveDuration = 0.2;

        //stripes
        public static final int stripesLength = 3;
        public static final double stripesDuration = 4.0;

        public static final double autoFadeMaxTime = 5.0; // Return to normal
        public static final double autoFadeTime = 2.5; // 3s nominal
        public static final double autoFastCycleLength = 15;
        public static final double autoFastDuration = 0.25;
  
      }

      public static final class Carriage{
        public static final int carriageFeederMotorID = 14;
        public static final int shooterFeederMotorID = 15;
        public static final int elevatorSensorChannel = 8;

        public static final double carriage_KP = 0.5; 
        public static final double carriage_KI = 0.0; 
        public static final double carriage_KD = 0.001; 
        public static final double carriage_KS = 0.0; 
        public static final double carriage_KV = 0.5; //0.5
        public static final double carriage_KA = 0.0;
        
        public static final double shooter_KP = 0.1; 
        public static final double shooter_KI = 0.0; 
        public static final double shooter_KD = 0.00; 
        public static final double shooter_KS = 0.6;//0.305; 
        public static final double shooter_KV = 0.925;//0.925; 
        public static final double shooter_KA = 0.0; 
        
        
        public static final double intakeVel_CarriageFeeder = 5.3;
        public static final double intakeVel_ShootingFeeder = 0.3;
        public static final double spitVel_CarriageFeeder = -14.0;
        public static final double spitVel_ShootingFeeder = -0.95;
        public static final double shootingVel_CarriageFeeder = 14.0;
        public static final double shootingVel_ShootingFeeder = 1.0;
        public static final double amp_scoring_CarriageShooterVel = 10.0;//14
        public static final double amp_scoring_ShootingFeederVel = -1.2;//-0.95

        public static final double carriageRollerDiameter=0.03;
        public static final double feederRollerDiameter=0.03;

        public static final double carriageFeederGearRatio=16.0/30.0;
        public static final double shooterFeederGearRatio=8.0;

        public static final int elevatorMotorID = 20;

        public static final double elevatorGearRatio = 15.0;
        public static final double Acc = 100.0;
        public static final double Vel = 80.0;
        public static final double Jer = 0.0;

        /* ElevatorConstants */
        public static final double ElevatorKP = 20.0; 
        public static final double ElevatorKI = 0.0;
        public static final double ElevatorKD = 0.03;
        public static final double ElevatorKS = 0.030909; 
        public static final double ElevatorKV = 1.6636;
        public static final double ElevatorKA = 0.020321;
        public static final double ElevatorKG = 0.12963;

        public static final double elevatorTrapPosition = 5.25;
        public static final double elevatorAmpPosition = 2.3;
        public static final double elevatorGroundPosition = 0.000001;

        public static final double elevatortolerance = 0.1;

        public static final double MinHeight = 0.0;
        public static final double MaxHeight = 5.30;
    }

    public static final class Climbing{
        public static final int climbtopMotorID = 21;
        public static final int climbbuttonMotorID = 22;

        public static final double climbGearRatio = 75.0;

        public static final double Acc = 80.0;
        public static final double Vel = 60.0;
        public static final double Jer = 0.0;

        public static final double climbKP = 40.0; 
        public static final double climbKI = 0.0;
        public static final double climbKD = 0.0;
        public static final double climbKS = 5.14185; 
        public static final double climbKV = 0.0;
        public static final double climbKA = 0.30801;
        public static final double climbKG = 0.30801;

        public static final double climbingForPos = 1.395;
        public static final double climbingBackPos = 0.0001;

        public static final double climbTolerance = 0.04;

        public static final double MinHeight = 0.0;
        public static final double MaxHeight = 1.41;

        public static final double FowardVel = -0.1;
        public static final double BackwardVel = 0.1;

    }
}
