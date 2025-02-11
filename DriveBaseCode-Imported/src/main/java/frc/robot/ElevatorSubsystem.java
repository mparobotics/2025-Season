// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID,MotorType.kBrushless);
  public RelativeEncoder encoder = elevatorMotor.getEncoder();
  /*public Command Run1(DoubleSupplier speed){
    return runOnce(()-> elevatorMotor.set(speed.getAsDouble()* Constants.slowMotorSpeedMultiplier));
    
  }*/ 

  /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
    //motor1.setInverted(false); -commented out bc unused
  }


  public Command RunMotors()
  {
return runOnce(
  () -> {
    elevatorMotor.set(0.5);
  }

);}

public Command StopMotors()
  {
return runOnce(
  () -> {
    elevatorMotor.set(0);
  }

);}

public Command InverseMotors()
  {
return runOnce(
  () -> {
    elevatorMotor.set(-0.5);
  }

);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
