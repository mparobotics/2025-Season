// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final SparkMax wristMotor = new SparkMax(WristConstants.wristMotorID,MotorType.kBrushless);
  public RelativeEncoder encoder = wristMotor.getEncoder();
  public WristSubsystem() {}
  public Command RunMotors()
  {
return runOnce(
  () -> {
    wristMotor.set(0.25);
  }

);}

  public Command StopMotors()
  {
  return runOnce(
  () -> {
    wristMotor.set(0);
  }

);}

  public Command InverseMotors()
  {
  return runOnce(
  () -> {
    wristMotor.set(-0.25);
  }

  );}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
