// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID,MotorType.kBrushless);
  public RelativeEncoder encoder = elevatorMotor.getEncoder();
  //PIDController pid = new PIDController(AutoConstants.k_elevatorP, AutoConstants.k_elevatorI, AutoConstants.k_elevatorD);
  //https://github.com/mparobotics/2024-Season/blob/main/2024%20Full%20Robot%20Code/src/main/java/frc/robot/subsystems/ArmSubsystem.java good example of PID

  public Command Run1(DoubleSupplier speed){
    if (speed.getAsDouble() > 1) {
      //TODO if greater than 1, make it 1
      throw new ArithmeticException("value greater than 1");
    } else if (speed.getAsDouble() < -1) {
      //TODO if less than -1, make it -1
      throw new ArithmeticException("value less than 1");
    }

    if (Math.abs(speed.getAsDouble()) < 0.2) {
      return runOnce(()-> elevatorMotor.set(0));
    } // FIX 0.2 is a placeholder, put in constants later


    return runOnce(()-> elevatorMotor.set(speed.getAsDouble()*0.25 + Constants.ElevatorConstants.elevatorFeedForward)); //0.25 is a placeholder
    
  }


  /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
    //motor1.setInverted(false); -commented out bc unused
  }

/* 
  public Command RunMotors()
  {
return runOnce(
  () -> {
    elevatorMotor.set(0.25); */
        /*Pose2d currentPose = new Pose2d (0.0, encoder.getPosition(), new Rotation2d(0.0)); //might not work
    Pose2d targetPose = m_goalPoseSupplier.get();
      
      
    double targetDirection = OnboardModuleState.closestAngle(currentDirection,targetPose.getRotation().getDegrees());

    double ySpeed = yController.calculate(currentPose.getY(),targetPose.getY()); */
//motor.set(pid.calculate(encoder.getDistance(), setpoint));
    //elevatorMotor.set(pid.calculate(encoder.getPosition(), AutoConstants.k_elevatorSetpoint));
  //}

//);} 
/* 
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
    elevatorMotor.set(-0.25);
  }

);} */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder",encoder.getPosition());

    //double ySpeed = yController.calculate(currentPose.getY(),targetPose.getY());
  }
}
