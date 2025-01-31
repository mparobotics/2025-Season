// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private final SparkCLosedLoopController angleController;

    private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
        SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error
 
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.m_angleKP = SwerveConstants.angleKP;
        this.m_angleKI = SwerveConstants.angleKI;
        this.m_angleKD = SwerveConstants.angleKD;
        this.m_angleKFF = SwerveConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

        angleEncoder.getAbsolutePosition().setUpdateFrequency(1);
        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
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

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        //might use more ...optimized... version if it works (needs testing)
        desiredState = 
        OnboardModuleState.optimize(desiredState, getState().angle);
        
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            //control the motor through direct open loop control
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            //control the motor through PID velocity controller with feedforward
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }


    private void setAngle(SwerveModuleState desiredState){
        //If we are moving at a very low speed, then there is no point in rotating the modules.
        // with zero speed, the target angle will be calculated as zero and the modules would point at zero every time you stop moving
        //if the speed is less than 1%, don't bother updating the angle
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; 
        
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
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
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    private void configAngleMotor(){
        //resets angle motor
        angleMotor.restoreFactoryDefaults();
        //limits can bus usage
        CANSparkUtil.setCANSparkBusUsage(angleMotor, Usage.kPositionOnly);
        //sets current limit
        angleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        //sets inversion
        angleMotor.setInverted(SwerveConstants.angleInvert);
        //sets brake mode or not
        angleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
        //sets a conversion factor for the encoder so it output correlates with the rotation of the module
        integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
        //oops pid loop time sets the pid
        angleController.setP(m_angleKP);
        angleController.setI(m_angleKI);
        angleController.setD(m_angleKD);
        angleController.setFF(m_angleKFF);
        angleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        //burns spark max
        angleMotor.burnFlash();

        Timer.delay(1.0);
        //resets to the cancoder
        resetToAbsolute();
    }

    private void configDriveMotor(){    
        //factory resets the spark max    
        driveMotor.restoreFactoryDefaults();
        //full utilisation on the can loop hell yea
        CANSparkUtil.setCANSparkBusUsage(driveMotor, Usage.kAll);
        //sets current limit
        driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        //sets inverted or not
        driveMotor.setInverted(SwerveConstants.driveInvert);
        //sets brake mode or not
        driveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
        //sets encoder to read velocities as meters per second
        driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        //sets encoder to read positions as meters traveled
        driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        //pid setting fun
        driveController.setP(SwerveConstants.driveKP);
        driveController.setI(SwerveConstants.driveKI);
        driveController.setD(SwerveConstants.driveKD);
        driveController.setFF(SwerveConstants.driveKFF);
        driveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        //burns to spark max
        driveMotor.burnFlash();
        //resets encoder position to 0
        driveEncoder.setPosition(0.0);
    }
}