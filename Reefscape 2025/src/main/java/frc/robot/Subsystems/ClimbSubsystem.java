// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final TalonFX climbMotor = new TalonFX(0);
  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private double m_setpoint = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor.getConfigurator().apply(motorConfiguration);
  }
  public void setTarget(double degrees){
    m_setpoint = degrees;
  }

  public Command changeArmSetpointCommand(double change){
    return runOnce(() -> { setTarget(m_setpoint + change); });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
