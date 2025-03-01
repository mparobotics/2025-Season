// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;

public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;

  private SwerveDrivePoseEstimator odometry;
  private SwerveModule[] swerveMods;

  private Field2d field;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration()); 
    zeroGyro();

    //Creates all four swerve modules into a swerve drive
    swerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    //creates new swerve odometry (odometry is where the robot is on the field)
    odometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(),new Pose2d(0,0,Rotation2d.fromDegrees(0)));

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }
  //translation: how fast robot moves in x and y directions; rotation: how fast robot spins
  //field relative: if controls are field or robot oriented
  public void drive(double x, double y,double rotation, boolean isFieldRelative)

  {
    //converts inputs to ChassisSpeeds
    driveFromChassisSpeeds(
          isFieldRelative //if field oriented conver desired speeds to robot oriented
          ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getYaw()) //if robot oriented generate ChassisSpeeds
          :new ChassisSpeeds(x, y, rotation) 
          , true); 
  
  }


  //set states for all 4 modules
  public void closedLoopDrive(ChassisSpeeds speeds){
    driveFromChassisSpeeds(speeds, false);
  }
  public void driveFromCHassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop){
    //get the target speed and direction for each module
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    //if any module is asked to drive at more than 100% speed, then scale ALL 4 speeds until the max speed of any module is 100%
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
        //set desired speed and direction for all 4 modules
        for (SwerveModule module : swerveMods) {
          module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
  }
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }


  public void resetOdometry(Pose2d pose) {
   odometry.resetPosition(getYaw(), getPositions(), pose);
  }
//set current heading to be zero degrees
  public void zeroGyro() {
      if(FieldConstants.isRedAlliance()){
        pigeon.setYaw(180);
      }
      else{
        pigeon.setYaw(0); 
      }
    }
  public Pose2d getpose() {
    return odometry.getEstimatedPosition();
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}
  
  public double[] getEncoderRotations() {
    double[] distances = new double[4];
    for (SwerveModule mod : swerveMods){
      distances[mod.moduleNumber] = mod.getRawDriveEncoder() / SwerveConstants.wheelCircumference;
    }
    return distances;
  }

  public double getYawAsDouble(){
    double yaw = pigeon.getAngle();
    return (SwerveConstants.invertPigeon)
    ? 360 - yaw 
    : yaw; 
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return Rotation2d.fromDegrees(getYawAsDouble());
  }
  public Translation2d getRelativeReefLocation(){
    Translation2d targetLocation = FieldConstants.isRedAlliance()? FieldConstants.RED_SPEAKER_LOCATION: FieldConstants.BLUE_SPEAKER_LOCATION;
    return targetLocation.minus(getpose().getTranslation());
  }
  public Translation2d getVirtualTarget(){
    Translation2d reefLocation = getRelativeReefLocation();
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeed(), getYaw());
    Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    
  }

  


  @Override
  public void periodic() {
        odometry.update(getYaw(), getPositions());
    field.setRobotPose(getpose());

    SmartDashboard.putNumber("Pigeon Roll",  pigeon.getPitch().getValueAsDouble());

    for (SwerveModule mod : swerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }
}

}