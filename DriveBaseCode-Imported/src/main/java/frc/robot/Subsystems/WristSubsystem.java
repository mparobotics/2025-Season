// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;


public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final SparkMax wristMotor = new SparkMax(WristConstants.wristMotorID,MotorType.kBrushless);
  private final SparkMaxConfig wristConfig = new SparkMaxConfig();
  public RelativeEncoder encoder = wristMotor.getEncoder();
  public WristSubsystem() {
    wristConfig.idleMode(IdleMode.kBrake); 
  }
  public Command RunWrist()
  {
return runOnce(
  () -> {
    wristMotor.set(0.125);
  }

);}

  public Command StopWrist()
  {
  return runOnce(
  () -> {
    wristMotor.set(0);
  }

);}

  public Command InverseWrist()
  {
  return runOnce(
  () -> {
    wristMotor.set(-0.125);
  }

  );}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //periodic: forever loop
    SmartDashboard.putNumber("wrist encoder",encoder.getPosition());
  }
}