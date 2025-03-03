// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CANSparkUtil;
import frc.lib.SwerveModuleConstants;
import frc.lib.CANSparkUtil.Usage;
import frc.robot.Constants.SwerveConstants;

/** A Single Swerve Module */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;

    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkFlex driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
  
    private CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;

    private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
        SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error
 
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

        angleEncoder.getAbsolutePosition().setUpdateFrequency(1);
        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),  getAngle()); 
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(),  getAngle()); 
    }
    public double getRawDriveEncoder(){
        return driveEncoder.getPosition();
    }
    public double getRawTurnEncoder(){
        return integratedAngleEncoder.getPosition();
    }
    //will only return true if neither motor has any errors. 
    public boolean isEncoderDataValid(){
        return driveMotor.getLastError() == REVLibError.kOk && angleMotor.getLastError() == REVLibError.kOk;
    }
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double difference = desiredState.angle.minus(currentAngle).getDegrees();
        double turnAmount = Math.IEEEremainder(difference,360);

        double speed = desiredState.speedMetersPerSecond;

        if (turnAmount > 90){
            turnAmount -= 180;
            speed *= -1;
        }
        if (turnAmount < -90){
            turnAmount += 180;
            speed *= -1;
        }
        return new SwerveModuleState (speed, currentAngle.plus(Rotation2d.fromDegrees(turnAmount)));
    }
        //might use more ...optimized... version if it works (needs testing)

         private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop){
         if (isOpenLoop) {
            double percentOutput = speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
         }
         else{
            double feedforwardOutput = feedforward.calculate(speedMetersPerSecond);
            driveController.setReference(speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardOutput);
         }
       } 

        public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState optimizedState = optimize(desiredState, getAngle());
        if (Math.abs(optimizedState.speedMetersPerSecond) > 0.01 * SwerveConstants.maxSpeed){
            setAngle(optimizedState.angle);
            
        }
        setSpeed(optimizedState.speedMetersPerSecond, isOpenLoop);
    }
 
    private void setAngle(Rotation2d angle){
     angleController.setReference(angle.getDegrees(), ControlType.kPosition);

    }
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }
    
    public void pointInDirection(double degrees){
        angleController.setReference(degrees, ControlType.kPosition);
        lastAngle = Rotation2d.fromDegrees(degrees);
    }


    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition); //may need to change 

      }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }

    private void configAngleMotor(){
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        //resets angle motor
        //angleMotor.restoreFactoryDefaults();
        //limits can bus usage
        CANSparkUtil.setSparkBusUsage(sparkMaxConfig, Usage.kPositionOnly);
        //sets current limit
        sparkMaxConfig.smartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        //sets inversion
        sparkMaxConfig.inverted(SwerveConstants.angleInvert);
        //sets brake mode or not
        sparkMaxConfig.idleMode(SwerveConstants.angleNeutralMode);
        //sets a conversion factor for the encoder so it output correlates with the rotation of the module
        sparkMaxConfig.encoder.positionConversionFactor(SwerveConstants.angleConversionFactor);
        //configuring pid, did not include feed forward
        sparkMaxConfig.closedLoop.p(m_angleKP).i(m_angleKI).d(m_angleKD);
        //TODO ADD FEED FORWARD
       // angleController.setFF(m_angleKFF);
        sparkMaxConfig.voltageCompensation(SwerveConstants.voltageComp);
        angleMotor.configure(sparkMaxConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        Timer.delay(1.0);
        //resets to the cancoder
        resetToAbsolute();
    }

    private void configDriveMotor(){    ;
        SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
        //factory resets the spark max    
        //full utilisation on the can loop hell yea
        CANSparkUtil.setSparkBusUsage(sparkFlexConfig, Usage.kAll);
        //sets current limit
        sparkFlexConfig.smartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        //sets inverted or not
        sparkFlexConfig.inverted(SwerveConstants.driveInvert);
        //sets brake mode or not
        sparkFlexConfig.idleMode(SwerveConstants.driveNeutralMode);
        //sets encoder to read velocities as meters per second
        sparkFlexConfig.encoder.velocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        //sets encoder to read positions as meters traveled
        sparkFlexConfig.encoder.positionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        //pid setting fun
        //configuring pid, did not include feed forward
        sparkFlexConfig.closedLoop.p(m_angleKP).i(m_angleKI).d(m_angleKD);
        //TODO ADD FEED FORWARD
        //driveController.setFF(SwerveConstants.driveKFF);
        sparkFlexConfig.voltageCompensation(SwerveConstants.voltageComp);
        //burns to spark max
        driveMotor.configure(sparkFlexConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }
}