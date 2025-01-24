package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ClawConstants;

public class ClawControlSubsystem extends SubsystemBase{
    
    private TalonFX GripMotor;
    private SparkMax intakeMotorLeft, intakeMotorRight;
    private SparkClosedLoopController intakeController; 


    private double position, desiredRotation, desiredVelocity;
    private MotionMagicDutyCycle motionControlRequest;

    public ClawControlSubsystem() {
        
        position = ClawConstants.CORAL_POSITION;

        GripMotor = new TalonFX(Ports.ARM_GRIP_MOTOR); 
        intakeMotorLeft = new SparkMax(Ports.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
        intakeMotorRight = new SparkMax(Ports.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ClawConstants.CLAW_STATOR_CURRENT_LIMIT;

        SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
        intakeFollowerConfig.follow(intakeMotorLeft.getDeviceId());
        intakeFollowerConfig.idleMode(IdleMode.kBrake);
        intakeFollowerConfig.smartCurrentLimit(40);
        intakeFollowerConfig.closedLoop.pidf(ClawConstants.ROLLER_KP, ClawConstants.ROLLER_KI, ClawConstants.ROLLER_KD, ClawConstants.ROLLER_KFF);

        SparkMaxConfig intakeMainConfig = new SparkMaxConfig();

        intakeMainConfig.follow(intakeMotorLeft.getDeviceId());
        intakeMainConfig.idleMode(IdleMode.kBrake);
        intakeMainConfig.smartCurrentLimit(40);
        intakeFollowerConfig.closedLoop.pidf(ClawConstants.ROLLER_KP, ClawConstants.ROLLER_KI, ClawConstants.ROLLER_KD, ClawConstants.ROLLER_KFF);


        intakeMotorLeft.configure(intakeMainConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        intakeMotorRight.configure(intakeFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

     
        talonFXConfigs.Slot0.kP = ClawConstants.CLAW_KP; 
        talonFXConfigs.Slot0.kI = ClawConstants.CLAW_KI; 
        talonFXConfigs.Slot0.kD = ClawConstants.CLAW_KD;
        talonFXConfigs.Slot0.kS = ClawConstants.CLAW_KS; 
        talonFXConfigs.Slot0.kV = ClawConstants.CLAW_KV;
        talonFXConfigs.Slot0.kA = ClawConstants.CLAW_KA; 
        talonFXConfigs.Slot0.kG = ClawConstants.CLAW_KG;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ClawConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.ACCELERATION;

        GripMotor.optimizeBusUtilization();
        GripMotor.getRotorPosition().setUpdateFrequency(20);

        GripMotor.getConfigurator().apply(talonFXConfigs);


        motionControlRequest = new MotionMagicDutyCycle(position);
        intakeController = intakeMotorLeft.getClosedLoopController();
    }

    @Override
    public void periodic() {
        if (GripMotor.hasResetOccurred())
		{
			GripMotor.optimizeBusUtilization();
			GripMotor.getRotorPosition().setUpdateFrequency(20);
		}

        
    }

    public void setClawMode(double position) {
        this.position = position;
        GripMotor.setControl(motionControlRequest);
    }

    public double getPosition() {
        return GripMotor.getRotorPosition().getValueAsDouble();
    }

    public void setDesiredVelocity(double velocity) {
        this.desiredVelocity = velocity;
        intakeController.setReference(velocity, SparkBase.ControlType.kVelocity);
    }

    public void setDesiredRotation(double rotation) {
        this.desiredRotation = rotation;
        intakeController.setReference(rotation, SparkBase.ControlType.kPosition);
    }

}
