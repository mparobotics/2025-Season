// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.TunableControllers.TunableArmFeedforward;
import frc.lib.TunableControllers.TunablePID;
import frc.robot.Constants.WristConstants;


public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final SparkMax wristMotor = new SparkMax(WristConstants.wristMotorID,MotorType.kBrushless);
  private RelativeEncoder encoder = wristMotor.getEncoder();

  private TunablePID wristPID = new TunablePID("wristPID", 0, 0, 0);
  private TunableArmFeedforward wristFeedforward = new TunableArmFeedforward("wristFeedforward", 0, 0, 0);
  private double SetpointAngle = 90;



  public WristSubsystem() {
  SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(false);
      config.idleMode(IdleMode.kBrake);
      config.encoder.positionConversionFactor(WristConstants.gearRatio);

    wristMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(90);
  }

  private void movetowardsSetpoints(){
    double output = wristFeedforward.calculate(Units.degreesToRadians(SetpointAngle),0) + 
    wristPID.calculate(encoder.getPosition(), SetpointAngle); //how fast motor go
    wristMotor.set(output); //need to fix positionradian
    SmartDashboard.putNumber("wristSetPoints", SetpointAngle);
    SmartDashboard.putNumber("actualVelocity", encoder.getVelocity());
    SmartDashboard.putNumber("actualPosition", encoder.getPosition());
    SmartDashboard.putNumber("motorOutput", output);
  }

  private void setSetpoint(double angle){
    SetpointAngle = angle;
  }

  public Command RunWrist()
  {
return runOnce(
  () -> {
    wristMotor.set(0.125);
  }

);}

  public Command StopWrist()
  {
  return runOnce(
  () -> {
    wristMotor.set(0);
  }

);}

  public Command InverseWrist()
  {
  return runOnce(
  () -> {
    wristMotor.set(-0.125);
  }
);}

  public Command setSetpointCommand(double angle)
  {
  return runOnce(
  ()->{
    setSetpoint(angle);
  }
);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //periodic: forever loop
    movetowardsSetpoints();
  }
}