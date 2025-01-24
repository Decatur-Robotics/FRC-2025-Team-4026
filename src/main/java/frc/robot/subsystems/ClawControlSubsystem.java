package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.ClawConstants;

public class ClawControlSubsystem extends SubsystemBase{
    
    private TalonFX clawMotor;
    private SparkMax intakeMotorLeft, intakeMotorRight;
    private SparkClosedLoopController intakeController; 


    private double position, velocity;
    private MotionMagicDutyCycle clawControlRequest;

    public ClawControlSubsystem() {
        clawMotor = new TalonFX(Ports.CLAW_MOTOR); 
        intakeMotorLeft = new SparkMax(Ports.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
        intakeMotorRight = new SparkMax(Ports.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ClawConstants.CLAW_STATOR_CURRENT_LIMIT;

        SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
        intakeFollowerConfig.idleMode(IdleMode.kBrake);
        intakeFollowerConfig.smartCurrentLimit(40);
        intakeFollowerConfig.follow(intakeMotorLeft.getDeviceId());
 
        SparkMaxConfig intakeMainConfig = new SparkMaxConfig();

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

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ClawConstants.CLAW_CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.CLAW_ACCELERATION;

        clawMotor.getConfigurator().apply(talonFXConfigs);

        clawMotor.optimizeBusUtilization();
        clawMotor.getRotorPosition().setUpdateFrequency(20);

        position = ClawConstants.CORAL_POSITION;

        clawControlRequest = new MotionMagicDutyCycle(position);
        clawMotor.setControl(clawControlRequest);

        velocity = ClawConstants.REST_VELOCITY;

        intakeController = intakeMotorLeft.getClosedLoopController();
        intakeController.setReference(velocity, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        if (clawMotor.hasResetOccurred())
		{
			clawMotor.optimizeBusUtilization();
			clawMotor.getRotorPosition().setUpdateFrequency(20);
		}

        
    }

    public void setClawPosition(double position) {
        this.position = position;
        clawMotor.setControl(clawControlRequest);
    }

    public double getClawPosition() {
        return clawMotor.getRotorPosition().getValueAsDouble();
    }

    public void setIntakeVelocity(double velocity) {
        this.velocity = velocity;
        intakeController.setReference(velocity, SparkBase.ControlType.kVelocity);
    }

   

}
