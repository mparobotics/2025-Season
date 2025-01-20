// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  /** Creates a new MotorSubsystem. */
  private final SparkMax motor = new SparkMax(0, MotorType.kBrushless);
  public RelativeEncoder motorEncoder = motor.getEncoder();

  public MotorSubsystem() {

  }


  public Command RunMotors()
  {
return runOnce(
  () -> {
    motor1.set(1);
  }

);}

public Command StopMotors()
  {
return runOnce(
  () -> {
    motor1.set(0);
  }

);}

public Command InverseMotors()
  {
return runOnce(
  () -> {
    motor1.set(-1);
  }

);}

public Command SetMotor(double speed)
  {
return runOnce(
  () -> {
    motor1.set(speed);
  }
);}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("motor speed", motorEncoder.getVelocity());
  }
}
