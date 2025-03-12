// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class Constants {
  public final class ElevatorConstants{
    public static final int elevatorMotorID = 16; //Placeholder, for test bot 
    public static final double slowMotorSpeedMultiplier = 0.5; 
    public static final double elevatorFeedForward = 0.01; 
  }
  public final class ClimberConstants{
    public static final int climbMotorID = 28;
  }
  public final class WristConstants{
    public static final int wristMotorID = 14;
  }
  public final class IntakeConstants{


  public static final int intakeMotorID = 15;
  
}

public static final double motorSpeedMultiplier = 1;

  public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final int PIGEON_ID = 17; //tochange
    public static final boolean invertPigeon = false;

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(22/2.0);//to find
    public static final double halfWheelBase = Units.inchesToMeters(22/2.0);//to find
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveBaseRadius = Math.hypot(halfTrackWidth/2, halfWheelBase/2);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1 L2 Mk4 Modules
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (21.4 / 1.0); // 12.8:1 MK4 SDS Modules
    //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1

    public static final SwerveDriveKinematics swerveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(halfTrackWidth, halfWheelBase), //translation 2d locates the swerve module in cords
        new Translation2d(halfTrackWidth,-halfWheelBase),
        new Translation2d(-halfTrackWidth,-halfWheelBase),
        new Translation2d(-halfTrackWidth, halfWheelBase));
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
    //SwerveDrive Kinematics converts between a ChassisSpeeds object and several SwerveModuleState objects, 
    //which contains velocities and angles for each swerve module of a swerve drive robot.
        
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
       
    //Swerve Current Limiting for neos
    public static final int angleContinuousCurrentLimit = 20; //limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 40; //limits current draw of drive motor
  


    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; //to tune
    public static final double driveKI = 0.0; //to tune
    public static final double driveKD = 0.0; //to tune
    public static final double driveKFF = 0.0; //to tune

    /* Drive Motor Characterization Values */
    //values to calculate the drive feedforward (KFF)
    public static final double driveKS = 0.667; //to calculate
    public static final double driveKV = 2.44; //to calculate
    public static final double driveKA = 0.27; //to calculate

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
    (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 5; // meters per second
    public static final double maxAngularVelocity = maxSpeed / driveBaseRadius; //radians per second how fast the robot spin

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean canCoderInvert = false;
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    //Location of modules
    public static final Translation2d FRONT_LEFT = new Translation2d(halfTrackWidth, halfWheelBase);
    public static final Translation2d BACK_LEFT = new Translation2d(-halfTrackWidth, halfWheelBase);
    public static final Translation2d BACK_RIGHT = new Translation2d(-halfTrackWidth, -halfWheelBase);
    public static final Translation2d FRONT_RIGHT = new Translation2d(halfTrackWidth, -halfWheelBase);
    /* Angle Encoder Invert */
    

        /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public record ModuleData(
      int driveMotorID, int angleMotorID, int encoderID, double angleOffset, Translation2d location
    ){}

    public static ModuleData[] moduleData = {
      new ModuleData(4, 2, 3, -141.24, FRONT_LEFT), //Mod 0
      new ModuleData(7, 5, 6, -69.69, FRONT_RIGHT), //Mod 1
      new ModuleData(10, 8, 9, 31.11, BACK_RIGHT), //Mod 2
      new ModuleData(13, 11, 12, 84.02, BACK_LEFT) //Mod 3
    };
      
    public static final double angleKP = 0.01; //to tune
    public static final double angleKI = 0.0; //to tune
    public static final double angleKD = 0.0; //to tune
    

    /*public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;*/
    
  

  }

  public static final class AutoConstants { //pathplanner
    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(SwerveConstants.wheelDiameter/2,
     SwerveConstants.maxSpeed, 
     1.2, 
     DCMotor.getNeoVortex(1).withReduction(SwerveConstants.driveGearRatio), 
     SwerveConstants.driveContinuousCurrentLimit, 
     1);
    
    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(52, 6.8, MODULE_CONFIG, 
    SwerveConstants.FRONT_LEFT, SwerveConstants.FRONT_RIGHT, SwerveConstants.BACK_LEFT, SwerveConstants.BACK_RIGHT);

    public static final PPHolonomicDriveController SWERVECONTROLLER = new PPHolonomicDriveController(new PIDConstants(5.0,0.00001,0.0), new PIDConstants(5.0, 0.0005, 0.001));
  }  

public class FieldConstants {
      public static final double FIELD_LENGTH = 17.54824934;
      public static final double FIELD_WIDTH = 8.052;

      public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.48933684,4.02587697);

      public static final Rotation2d RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(234.011392);
      public static final Rotation2d LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-234.011392);

      public static boolean isRedAlliance(){
          return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
      }
      
      public static Rotation2d flipForAlliance(Rotation2d rotation){
          if(isRedAlliance()){
              return Rotation2d.fromDegrees(rotation.getDegrees() + 180);
          }else{
              return rotation;
          }
      }
      public static Translation2d flipForAlliance(Translation2d pos){
          if(isRedAlliance()){
              return new Translation2d(FIELD_LENGTH - pos.getX(), FIELD_WIDTH - pos.getY());
          }else{
              return pos;
          }
      }
      public static Pose2d flipForAlliance(Pose2d pose){
          return new Pose2d(flipForAlliance(pose.getTranslation()), flipForAlliance(pose.getRotation()));
      }
  }
  
}