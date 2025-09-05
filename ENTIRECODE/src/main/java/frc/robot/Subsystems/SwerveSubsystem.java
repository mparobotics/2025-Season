// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleData;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;

  private SwerveDrivePoseEstimator odometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private final StructArrayPublisher<SwerveModuleState> swerveDataPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> desiredSwerveDataPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Desired Swerve States", SwerveModuleState.struct).publish();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() { 
    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration()); 
    zeroGyro();

    //Creates all four swerve modules into a swerve drive
    mSwerveMods = new SwerveModule[4];
    for (int i = 0; i < 4; i++){
      ModuleData data = SwerveConstants.moduleData[i];
      mSwerveMods[i] = new SwerveModule(i, data);
    }

    //creates new swerve odometry (odometry is where the robot is on the field)
    odometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(), new Pose2d());

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }
  


  private void updateOdometryWithVision (String limelightName){
    boolean doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(limelightName, odometry.getEstimatedPosition().getRotation().getDegrees(),0,0,0,0,0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
      if (mt2 == null){
        return;
      }
      if(Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble())> 720)
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill (.7,.7,99999));// need to measure
        odometry.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
      }
    }


  public void drive(double xInput, double yInput, double rotationInput, boolean isFieldOriented){
    ChassisSpeeds desiredSpeeds;

    if(isFieldOriented){
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xInput, yInput, rotationInput, getYaw());
    }
    else{
      desiredSpeeds = new ChassisSpeeds(xInput, yInput, rotationInput);
    }
    driveFromChassisSpeeds(desiredSpeeds, true);
  }
 
  public void driveFromChassisSpeeds(ChassisSpeeds driveSpeeds, boolean isOpenLoop){
    SwerveModuleState[] desiredStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(driveSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    desiredSwerveDataPublisher.set(desiredStates);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public ChassisSpeeds getChassisSpeeds(){
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}
  
  public double[] getEncoderRotations() {
    double[] distances = new double[4];
    for (SwerveModule mod : mSwerveMods){
      distances[mod.moduleNumber] = mod.getRawDriveEncoder() / SwerveConstants.wheelCircumference;
    }
    return distances;
  }

  public void zeroGyro() {
    if (FieldConstants.isRedAlliance()){
      pigeon.setYaw(180);
    }
    else {
      pigeon.setYaw(0);
    }
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertPigeon)
        ? Rotation2d.fromDegrees(360 - pigeon.getYaw().getValueAsDouble())
        : Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }



  @Override
  public void periodic() {
        odometry.update(getYaw(), getPositions());
        updateOdometryWithVision("limelight-a");
        updateOdometryWithVision("limelight-b");
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon Yaw",  pigeon.getYaw().getValueAsDouble());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }
  swerveDataPublisher.set(getStates());
}

}