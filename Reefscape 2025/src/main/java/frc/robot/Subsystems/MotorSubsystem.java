// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

//import frc.robot.Constants; -commented out bc unused-
import frc.robot.Constants.ShooterConstants;




public class MotorSubsystem extends SubsystemBase {
  private final SparkMax motor1 = new SparkMax(ShooterConstants.MOTOR_1_ID,MotorType.kBrushless);
  /** Creates a new MotorSubsystem. */
public RelativeEncoder encoder = motor1.getEncoder();

  public MotorSubsystem() {
    //motor1.setInverted(false); -commented out bc unused
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    SmartDashboard.putNumber("Encoder Speed", encoder.getVelocity());
  }
}
