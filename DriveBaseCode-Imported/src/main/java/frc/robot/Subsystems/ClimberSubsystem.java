// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystens;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubystem. */
  
  private final SparkMax climbMotor = new SparkMax(ClimberConstants.climbMotorID,MotorType.kBrushless);
  public RelativeEncoder encoder = climbMotor.getEncoder();

  public ClimberSubsystem() {}

  public Command RunMotors()
  {
    return runOnce(
      () -> {
        climbMotor.set(0.25);
      }

    ); }

    public Command StopMotors()
    {
      return runOnce(
        () -> {
          climbMotor.set(0);
        }
  
      ); }
      
      public Command InverseMotors()
  {
    return runOnce(
      () -> {
        climbMotor.set(-0.25);
      }

    ); }
      



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
