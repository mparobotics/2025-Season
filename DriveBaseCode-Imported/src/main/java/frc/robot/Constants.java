// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

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
    public static final double trackWidth = Units.inchesToMeters(17.5);//to find
    public static final double wheelBase = Units.inchesToMeters(20.5);//to find
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveBaseRadius = Math.hypot(trackWidth/2, wheelBase/2);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1 L2 Mk4 Modules
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (21.4 / 1.0); // 12.8:1 MK4 SDS Modules
    //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1

    public static final SwerveDriveKinematics swerveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0,trackWidth / 2.0), //translation 2d locates the swerve module in cords
        new Translation2d(-wheelBase / 2.0,trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0,-trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0,-trackWidth / 2.0));
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
    public static final double maxSpeed = 9; // meters per second
    public static final double maxAngularVelocity = 11.5; //what are these units?

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean canCoderInvert = false;
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;
    /* Angle Encoder Invert */
    

        /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 4; 
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
    /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.2);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
        
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;                     
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
  
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
        //creates a constant with all info from swerve module
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 11; //spark max
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.256);
        /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;
    
  

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double k_elevatorP = 0.5;
    public static final double k_elevatorI = 0;
    public static final double k_elevatorD = 0;
    public static final double k_elevatorSetpoint = 0.5;

    

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
}