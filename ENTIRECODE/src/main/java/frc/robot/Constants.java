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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public final class Constants {
  public final class ElevatorConstants{
    public static final int elevatorMotorID = 16; //CAN ID for elevator motor controller
    public static final double slowMotorSpeedMultiplier = 0.5; // Multiplier to slow elevator speed when needed
    public static final double elevatorFeedForward = 0.01; // Feedforward value for elevator motor control
  
  // Constraints for trapezoidal motion profiling (max velocity and acceleration)
  public static final double maxVelocity = 1; // Maximum elevator velocity when profiling (m/s)
  public static final double maxAcceleration = 1; // Maximum elevator acceleration when profiling (m/s^2)
  public static final TrapezoidProfile.Constraints CONSTRAINTS = new Constraints(maxVelocity, maxAcceleration); // Shared trapezoidal profile limits

  // Gear and mechanical propertiees for converting motor rotations to elevator movement distance
  public static final double gearRatio = 12; // Overall reduction from motor to carriage motion
  public static final double pitchDiameter = 0.0447; // Diameter of gear pitch circle in meters
  public static final double circumference = pitchDiameter * Math.PI; // Circumference of gear pitch circle
  public static final double positionConversionFactor = circumference/gearRatio; // Converts motor rottions to elevator linear movement(meters)
  public static final double velocityConversionFactor = circumference/gearRatio/60; // Converts motor RPM to linear speed (m/s)

  public static final double closeEnough = 0.01; // Allowed tolerance for elevator position control (meters)
  }


  // Climber Constants
  public final class ClimberConstants{
    public static final int climbMotorID = 28; //CAN ID for climb motor controller
  }


  // Wrist Constants
  public final class WristConstants{
    public static final int wristMotorID = 15; // CAN ID for wrist motor controller
    // Gear ratio calculation combining different gear stages (e.g., 25 8 (50/34))
    public static final double gearRatio = 25*(50.0/34);

    public static final double closeEnough = 2; // Angle tolerance in degrees for wrist positioning
  }


  // Intake Constants
  public final class IntakeConstants{
    //limit switch
    public static final int IntakeSwitchPort = 0; //Digital input port for intake limit switch (placeholder)
  public static final int intakeMotorID = 14; // CAN ID for intake motor controller
  
}

public final class ScoreAngle{
  public record ScoringPose(
      double elevatorheight, double wristangle
    ){} 
  // Pair of elevator height (meters) and wrist angle (degrees) for presets
  public static final ScoringPose L1 = new ScoringPose (0.20, 3.74); // Level 1 reef scoring preset
  public static final ScoringPose L2 = new ScoringPose (0.59, -27.5); // Level 2 reef scoring preset
  public static final ScoringPose L3 = new ScoringPose (0.87, -24); // Level 3 reef scoring preset
  public static final ScoringPose L4 = new ScoringPose (0.88, 55); // Level 4 reef scoring preset

  public static final ScoringPose KnockAlgae = new ScoringPose (0.34, 30); // Pose used to knock algae off the reef

  public static final ScoringPose INTAKE = new ScoringPose(0.33, 39); // Coral intake pose

  public static final ScoringPose MOVE = new ScoringPose(0, 80); // moving pose
}

public static final double motorSpeedMultiplier = 0.5; // Used to scale down motor output if needed


// Swerve Constants
  public static final class SwerveConstants{
    public static final double inputDeadband = .1; // Deadzone for joystick inputs to prevent drift
    public static final int PIGEON_ID = 17; //CAN ID for Pigeon gyro sensor
    public static final boolean invertPigeon = false; // Whether to invert gyro readings

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(22/2.0); // Half the lateral distance between wheel modules
    public static final double halfWheelBase = Units.inchesToMeters(22/2.0); // Half the longitudinal distance between wheel modules
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // Diameter of each swerve wheel (meters)
    public static final double wheelCircumference = wheelDiameter * Math.PI; // Distance traveled per wheel rotation
    public static final double driveBaseRadius = Math.hypot(halfTrackWidth/2, halfWheelBase/2); // Effective robot radius for turning calculations

    public static final double openLoopRamp = 0.25; // Seconds for drive motors to ramp in open-loop mode
    public static final double closedLoopRamp = 0.0; // Seconds for drive motors to ramp in closed-loop mode

    public static final double driveGearRatio = (8.14 / 1.0); 
    /* overall reduction between the drive motor shaft and the wheel. 
    8.14:1 means the motor spins 8.14 revolutions for every single wheel revolution.*/ 
    public static final double angleGearRatio = (21.4 / 1.0); 
    /*reduction in the steering assembly. 
    21.4:1 means the steering motor must turn 21.4 times to rotate the wheel module exactly once around its axis */

    public static final SwerveDriveKinematics swerveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(halfTrackWidth, halfWheelBase), //translation 2d locates the swerve module in cords
        new Translation2d(halfTrackWidth,-halfWheelBase),
        new Translation2d(-halfTrackWidth,-halfWheelBase),
        new Translation2d(-halfTrackWidth, halfWheelBase));
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
    /*SwerveDrive Kinematics converts between a ChassisSpeeds object and several SwerveModuleState objects, 
    which contains velocities and angles for each swerve module of a swerve drive robot.*/
        
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
    (wheelDiameter * Math.PI) / driveGearRatio; // Converts drive motor rotations to meters traveled
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0; // Converts drive velocity RPM to m/s
    public static final double angleConversionFactor = 360.0 / angleGearRatio; // Converts angle motor rotations to module heading degrees

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3; // meters per second
    public static final double maxAngularVelocity = maxSpeed/driveBaseRadius; //radians per second how fast the robot spin

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake; // Lock steering motors when idle
    public static final IdleMode driveNeutralMode = IdleMode.kBrake; // Resist rolling when drive motors idle

    /* Motor Inverts */
    public static final boolean canCoderInvert = false; // Absolute encoder orientation
    public static final boolean driveInvert = false; // Drive motor orientation
    public static final boolean angleInvert = true; // Steering motor orientation

    //Location of modules
    public static final Translation2d FRONT_LEFT = new Translation2d(halfTrackWidth, halfWheelBase); 
    public static final Translation2d BACK_LEFT = new Translation2d(-halfTrackWidth, halfWheelBase); 
    public static final Translation2d BACK_RIGHT = new Translation2d(-halfTrackWidth, -halfWheelBase); 
    public static final Translation2d FRONT_RIGHT = new Translation2d(halfTrackWidth, -halfWheelBase); 
    /* Angle Encoder Invert */
    

        /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    // Record bundling hardware IDs, calibration, and physical location for a single module
    public record ModuleData(
      int driveMotorID, int angleMotorID, int encoderID, double angleOffset, Translation2d location
    ){}

    // Indexed configuration for each physical swerve module
    public static ModuleData[] moduleData = { 
      new ModuleData(4, 2, 3, 271.95, FRONT_LEFT), //Mod 0
      new ModuleData(7, 5, 6, 88.24, FRONT_RIGHT), //Mod 1
      new ModuleData(10, 8, 9, 212.61, BACK_RIGHT), //Mod 2
      new ModuleData(13, 11, 12, 15.20, BACK_LEFT) //Mod 3
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
    // Robot-wide parameters for PathPlanner
    
    public static final PPHolonomicDriveController SWERVECONTROLLER = new PPHolonomicDriveController(new PIDConstants(5.0,0.00001,0.0), new PIDConstants(5.0, 0.0005, 0.001)); 
    // Holonomic controller used for autonomous path tracking
    
    public enum AutoMode{
      LEAVE_AUTO, // Simple leave-community routine
      ONECORAL_AUTO, // Scores a single coral on default level
      ONECORAL_AUTO_L2, // Scores one coral on level 2
      TWOCORAL_AUTO, // Runs a two-coral auto sequence
      KNOCKALGAEOFF // Focuses on removing algae during auto
    }
    private static SendableChooser<Boolean> sideChooser = new SendableChooser<Boolean>(); // Dashboard chooser to select right/left starting side
    private static SendableChooser<AutoMode> autoModeChooser = new SendableChooser<AutoMode>(); // Dashboard chooser for main auto routine
    private static SendableChooser<AutoMode> leaveAutoChooser = new SendableChooser<AutoMode>(); // Dashboard chooser for leave-only routines
    static{
      sideChooser.addOption("RIGHT", true); // Provide option for right-side autos
      sideChooser.setDefaultOption("LEFT", false); // Default to left-side autos

      for(AutoMode mode : AutoMode.values()){ 
        autoModeChooser.addOption(mode.toString(), mode); // Publish each auto routine choice
      }
      //autoModeChooser.setDefaultOption("LEAVE_AUTO", AutoMode.LEAVE_AUTO);
      SmartDashboard.putData("LEAVE_AUTO_CHOOSER", leaveAutoChooser); // Display leave auto chooser on dashboard
      SmartDashboard.putData("Auto Starting Location", sideChooser); // Display side chooser on dashboard
      SmartDashboard.putData("Auto Mode", autoModeChooser); // Display primary auto chooser on dashboard
    }
    public static AutoMode getSelectedAuto(){
      return autoModeChooser.getSelected(); // Retrieve currently selected auto routine
    }
    public static boolean isRightSideAuto(){
      return sideChooser.getSelected(); // Determine whether autos should mirror for right side
    }
  }

// Field layout constants and alliance mirroring helpers
public class FieldConstants {
      public static final double FIELD_LENGTH = 17.54824934; // Total official field length (meters)
      public static final double FIELD_WIDTH = 8.052; // Total official field width (meters)

      public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.48933684,4.02587697); 
      // Center point of the blue alliance reef

      public static final Rotation2d RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(234.011392); 
      // Heading required to face right coral station
      public static final Rotation2d LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-234.011392); 
      // Heading required to face left coral station

      public static boolean isRedAlliance(){
          return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red; 
          // True when FMS reports we are on red
      }
      
      public static Rotation2d flipForAlliance(Rotation2d rotation){
          if(isRedAlliance()){
              return Rotation2d.fromDegrees(rotation.getDegrees() + 180); 
              // Rotate headings 180° for red alliance perspective
          }else{
              return rotation;
          }
      }
      public static Translation2d flipForAlliance(Translation2d pos){
          if(isRedAlliance()){
              return new Translation2d(FIELD_LENGTH - pos.getX(), FIELD_WIDTH - pos.getY()); 
              // Mirror X/Y positions across the field centerline
          }else{
              return pos;
          }
      }
      public static Pose2d flipForAlliance(Pose2d pose){
          return new Pose2d(flipForAlliance(pose.getTranslation()), flipForAlliance(pose.getRotation())); 
          // Mirror full pose for the opposing alliance
      }
  }
  
}
