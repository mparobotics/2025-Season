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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CANSparkUtil;
import frc.lib.CANSparkUtil.Usage;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleData;


/** A Single Swerve Module */
public class SwerveModule {
    public int moduleNumber;
    private double m_angleKP; // Proportional gain for steering loop
    private double m_angleKI; // Integral gain for steering loop
    private double m_angleKD; // Derivative gain for steering loop

    private Rotation2d lastAngle; // Cached last commanded steering angle
    private Rotation2d angleOffset; // Absolute encoder offset for module alignment

    private SparkMax angleMotor; // Steering motor controller
    private SparkFlex driveMotor; // Drive motor controller

    private RelativeEncoder driveEncoder; // Integrated drive encoder
    private RelativeEncoder integratedAngleEncoder; // SparkMAX internal steering encoder
  
    private CANcoder angleEncoder; // Absolute CANcoder for steering calibration

    private final SparkClosedLoopController driveController; // Closed-loop interface for drive motor
    private final SparkClosedLoopController angleController; // Closed-loop interface for steering motor

    private final SimpleMotorFeedforward feedforward =
    new SimpleMotorFeedforward(
        SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
    //creates a feedforward for the swerve drive. feedforward does 90% of the work, estimating stuff
    //PID fixes the error
 
    public SwerveModule(int moduleNumber, ModuleData moduleConstants){
        this.moduleNumber = moduleNumber; // Store module index for lookup
        this.m_angleKP = SwerveConstants.angleKP; // Load steering P gain
        this.m_angleKI = SwerveConstants.angleKI; // Load steering I gain
        this.m_angleKD = SwerveConstants.angleKD; // Load steering D gain
        angleOffset = Rotation2d.fromDegrees(moduleConstants.angleOffset()); // Convert calibration offset to Rotation2d
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.encoderID()); // Create CANcoder using configured ID
        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration()); // Apply default sensor config

        angleEncoder.getAbsolutePosition().setUpdateFrequency(1); // Limit network update rate to reduce CAN load
        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID(), MotorType.kBrushless); // Instantiate steering SparkMAX
        integratedAngleEncoder = angleMotor.getEncoder(); // Grab integrated encoder reference
        angleController = angleMotor.getClosedLoopController(); // Cache steering PID controller
        configAngleMotor(); // Apply steering motor configuration

        /* Drive Motor Config */
        driveMotor = new SparkFlex(moduleConstants.driveMotorID(), MotorType.kBrushless); // Instantiate drive Spark Flex
        driveEncoder = driveMotor.getEncoder(); // Grab drive encoder reference
        driveController = driveMotor.getClosedLoopController(); // Cache drive PID controller
        configDriveMotor(); // Apply drive motor configuration

        lastAngle = getState().angle; // Initialize lastAngle to current measurement
    }

    // Report instantaneous module velocity and steering angle
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),  getAngle()); // Combined current module velocity and angle
    }
    // Report cumulative drive distance and current steering angle
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(),  getAngle()); // Distance driven with current angle
    }
    public double getRawDriveEncoder(){
        return driveEncoder.getPosition(); // Raw drive encoder rotations
    }
    public double getRawTurnEncoder(){
        return integratedAngleEncoder.getPosition(); // Raw steering encoder position
    }
    //will only return true if neither motor has any errors. 
    public boolean isEncoderDataValid(){
        return driveMotor.getLastError() == REVLibError.kOk && angleMotor.getLastError() == REVLibError.kOk;
    }
    
    // Minimize required steering rotation by potentially reversing wheel direction
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double difference = desiredState.angle.getDegrees() - currentAngle.getDegrees(); // Compute angular error relative to current heading
        double turnAmount = Math.IEEEremainder(difference,360); // Wrap difference to ±180°

        double speed = desiredState.speedMetersPerSecond; // Start with requested speed

        if (turnAmount > 90){
            turnAmount -= 180; // Flip direction when faster to drive backwards
            speed *= -1; // Reverse wheel direction to maintain net heading
        }
        if (turnAmount < -90){  //was -90
            turnAmount += 180; //was 180
            speed *= -1;
        }

        double direction = currentAngle.getDegrees() + turnAmount; // Target absolute angle after optimization
        return new SwerveModuleState (speed, Rotation2d.fromDegrees(direction)); // Optimized state minimizing steering rotation
    }

    // Apply wheel speed using either open-loop percentage or closed-loop velocity control
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if (isOpenLoop) {
            //controls motor through openlooop control directly
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else{
        
            driveController.setReference(
                desiredState.speedMetersPerSecond, 
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(desiredState.speedMetersPerSecond)); // Run closed-loop velocity command with feedforward
                
        }
    }
    // Accept a desired state, optimize it, then command steering and drive outputs
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState optimizedState = optimize(desiredState, getAngle()); // Minimize steering movement before commanding
        setAngle(optimizedState); // Command steering to target heading
        setSpeed(optimizedState, isOpenLoop); // Apply speed command using chosen control mode
    }
         



    
 
    // Command the steering motor to the desired angle while handling low-speed hold
    private void setAngle(SwerveModuleState desiredState){
     Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
        ? lastAngle : desiredState.angle; // Hold last angle at very low speeds to prevent jitter
     angleController.setReference(angle.getDegrees(), ControlType.kPosition); // Command steering motor to target heading
     lastAngle = angle; // Cache commanded angle for next cycle

    }
    // Return the current steering angle based on integrated encoder feedback
    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition()); // Current steering angle derived from integrated encoder
    }
    
    // Manually point the module toward a specified heading
    public void pointInDirection(double degrees){
        angleController.setReference(degrees, ControlType.kPosition); // Force module toward supplied heading
        lastAngle = Rotation2d.fromDegrees(degrees); // Remember manual angle setpoint
    }


    // Synchronize the integrated steering encoder with the absolute CANcoder
    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees(); // Calculate zeroed steering position
        integratedAngleEncoder.setPosition(absolutePosition); //may need to change 

      }
    
    // Read the raw absolute steering angle provided by the CANcoder
    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Units.Rotations)); // Read absolute angle from CANcoder
    }

    // Configure all steering motor parameters (limits, PID, conversions)
    private void configAngleMotor(){
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig(); // Configuration container for steering SparkMAX
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
       // angleController.setFF(m_angleKFF);
        sparkMaxConfig.voltageCompensation(SwerveConstants.voltageComp);
        angleMotor.configure(sparkMaxConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        Timer.delay(1.0); // Give hardware time to apply configuration before reading sensors
        //resets to the cancoder
        resetToAbsolute();
    }

    // Configure drive motor parameters, conversion factors, and PID gains
    private void configDriveMotor(){    ;
        SparkFlexConfig sparkFlexConfig = new SparkFlexConfig(); // Configuration container for drive Spark Flex
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
        /* 
        driveController.setP(SwerveConstants.driveKP);
        driveController.setI(SwerveConstants.driveKI);
        driveController.setD(SwerveConstants.driveKD);
        driveController.setKFF(SwerveConstants.driveKFF);
        driveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);*/

        
        //configuring pid, did not include feed forward
        sparkFlexConfig.closedLoop.p(m_angleKP).i(m_angleKI).d(m_angleKD);
        //driveController.setFF(SwerveConstants.driveKFF);
        sparkFlexConfig.voltageCompensation(SwerveConstants.voltageComp);
        driveMotor.configure(sparkFlexConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //resets encoder position to 0
        driveEncoder.setPosition(0.0); // Start drive encoder counts at zero on boot
    }
}
