// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.TunableControllers.TunableElevatorFeedforward;
import frc.lib.TunableControllers.TunablePID;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID,MotorType.kBrushless);
  private RelativeEncoder encoder = elevatorMotor.getEncoder();

  private TunablePID elevatorPID = new TunablePID("elevatorPID", 2, 0, 0);
  private TunableElevatorFeedforward elevatorFeedforward = new TunableElevatorFeedforward("elevatorFeedforward", 0, 0.02, 0.8);
  private TrapezoidProfile profile = new TrapezoidProfile(ElevatorConstants.CONSTRAINTS);
  private TrapezoidProfile.State startingState;
  private TrapezoidProfile.State goalState;
  private double setpoint;
  private Timer timer = new Timer();


  /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
      SparkMaxConfig config = new SparkMaxConfig();
      setSetpoint(encoder.getPosition());
      config.inverted(true);
      config.idleMode(IdleMode.kBrake);
      config.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);
      config.encoder.velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

      elevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //wish it works
  }

  private void movetowardsSetpoints(){
    TrapezoidProfile.State desiredState = profile.calculate(timer.get(), startingState, goalState);
    double output = elevatorFeedforward.calculate(desiredState.velocity) + elevatorPID.calculate(encoder.getPosition(), desiredState.position); //how fast motor go
    elevatorMotor.set(output);
    SmartDashboard.putNumber("elevatorSetPoints", setpoint);
    SmartDashboard.putNumber("desiredVelocity", desiredState.velocity);
    SmartDashboard.putNumber("desiredPosition", desiredState.position);
    SmartDashboard.putNumber("actualVelocity", encoder.getVelocity());
    SmartDashboard.putNumber("actualPosition", encoder.getPosition());
    SmartDashboard.putNumber("motorOutput", output);
  }

  private void setSetpoint(double height){
    startingState = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
    goalState = new TrapezoidProfile.State(height, 0);
    setpoint = height;
    timer.restart();
  }

  public Command setSetpointCommand (double height){
    return runOnce(() -> setSetpoint (height));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    movetowardsSetpoints();
    elevatorPID.refresh();
    elevatorFeedforward.refresh();
  }
}
