// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorID,MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
public RelativeEncoder encoder = intakeMotor.getEncoder();
public Command Run1(DoubleSupplier speed){
return runOnce(() -> intakeMotor.set(speed.getAsDouble() * Constants.motorSpeedMultiplier)); } 
  public IntakeSubsystem() {
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
