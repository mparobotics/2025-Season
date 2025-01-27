// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;



public class MotorSubsystem extends SubsystemBase {
  private final SparkMax motor1 = new SparkMax(ShooterConstants.MOTOR_1_ID,MotorType.kBrushless);
 // private final SparkMax motor2 = new SparkMax (ShooterConstants.MOTOR_2_ID,MotorType.kBrushless);
  /** Creates a new MotorSubsystem. */
public RelativeEncoder encoder = motor1.getEncoder();
//public RelativeEncoder encoder2 = motor2. getEncoder();
public Command Run1(DoubleSupplier speed){
return runOnce(()-> motor1.set(speed.getAsDouble() * Constants.motorSpeedMultiplier));
SmartDashboard.putNumber("Speed Double", speed);
} 

public MotorSubsystem(){
  //motor1.setInverted(false);
 // motor2.setInverted(true);
 } 

 public void RunMotors(double speed){
  motor1.set(speed);
 }

  public Command MotorControl(DoubleSupplier speed){
return runOnce(() -> {
  RunMotors(MathUtil.applyDeadband(speed.getAsDouble(), 0.2));
    //motor2.set(Axis.kLeftY.value);
    
  }

);}

public Command StopMotors()
  {
return runOnce(
  () -> {
    motor1.set(0);
   // motor2.set(0);
  }

);
}

// public Command InverseMotors()
//   {
// return runOnce(
//   () -> {
//     motor1.set(Axis.kRightY.value * -1);
//    // motor2.set(Axis.kLeftY.value);
//   }

// );}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    SmartDashboard.putNumber("Encoder Speed", encoder.getVelocity());
  }
}
