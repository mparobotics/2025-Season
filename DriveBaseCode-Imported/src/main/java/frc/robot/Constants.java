// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
 public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final int PIGEON_ID = 17; 
    public static final boolean invertPigeon = true;

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(21.0 / 2.0); //half of the left-right distance between the wheels
    public static final double halfWheelBase = Units.inchesToMeters(21.0 /2.0 ) ; //half of the forward-backward distance between the wheels
    public static final double driveBaseRadius =  Math.hypot(halfTrackWidth,halfWheelBase);

    public static final double wheelDiameter = (.0992);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    

    public static final double driveGearRatio = (8.14 / 1.0); // 8.14:1 ( SDS Mk4 L1 Module )
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (12.4 / 1.0); // 12.8:1 ( SDS Mk4 L1 Module ) 
    //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1
    

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelCircumference) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
 
    /* Maximum speed and angular velocity of the robot */
    public static final double maxSpeed = 5; // meters per second
    public static final double maxAngularVelocity = maxSpeed / driveBaseRadius; //radians per second

    //give location of each module to a swerveDriveKinematics relative to robot center in meters
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(halfWheelBase, halfTrackWidth), 
      new Translation2d(-halfWheelBase, halfTrackWidth),
      new Translation2d(-halfWheelBase, -halfTrackWidth),
      new Translation2d(halfWheelBase, -halfTrackWidth)
    );
    public static final double voltageComp = 12.0;

    public static final int angleContinuousCurrentLimit = 20; //limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 50; //limits current draw of drive motor
  
    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; 
    public static final double driveKI = 0.0; 
    public static final double driveKD = 0.0; 
   public static final double driveKFF = 0.0; 

    /* Drive Motor Feedforward Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44; 
    public static final double driveKA = 0.5; //previously 0.27

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01; 
    public static final double angleKI = 0.0; 
    public static final double angleKD = 0.0; 
    public static final double angleKFF = 0.0; 
  

   

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;
    
    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3; 
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.2);
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }
    /* Back Left Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(117.2);
        
          
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            //creates a constant with all info from swerve module
      }
       /* Back Right Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 13;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141);
        
    
        public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
      }
        /* Front Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 9;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 14 ;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-138);
         
      
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            //creates a constant with all info from swerve module
      }

}
}
