package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ClimberConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX climberMotorRight, climberMotorLeft;
    
    private double position;
    private MotionMagicDutyCycle motorControlRequest;
    
    public ClimberSubsystem() {

        position = ClimberConstants.CLIMBER_POSITION_REST;

        climberMotorRight = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT);
        climberMotorLeft = new TalonFX(Ports.CLIMBER_MOTOR_LEFT);

        climberMotorRight.setControl(new Follower(Ports.CLIMBER_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = ClimberConstants.CLIMBER_MOTOR_KS; 
        slot0Configs.kV = ClimberConstants.CLIMBER_MOTOR_KV;
        slot0Configs.kA = ClimberConstants.CLIMBER_MOTOR_KA; 
        slot0Configs.kP = ClimberConstants.CLIMBER_MOTOR_KP; 
        slot0Configs.kI = ClimberConstants.CLIMBER_MOTOR_KI; 
        slot0Configs.kD = ClimberConstants.CLIMBER_MOTOR_KD;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.MOTION_MAGIC_ACCELERATION;
        talonFXConfigs.MotionMagic.MotionMagicJerk = ClimberConstants.MOTION_MAGIC_JERK;

        climberMotorLeft.getConfigurator().apply(talonFXConfigs);
        climberMotorRight.getConfigurator().apply(talonFXConfigs);

        climberMotorLeft.optimizeBusUtilization();
        climberMotorRight.optimizeBusUtilization();
        climberMotorLeft.getRotorPosition().setUpdateFrequency(20);
        climberMotorRight.getRotorPosition().setUpdateFrequency(20);

        motorControlRequest = new MotionMagicDutyCycle(position);
        climberMotorLeft.setControl(motorControlRequest);

        
    }

    @Override
    public void periodic() {
        if (climberMotorLeft.hasResetOccurred() || climberMotorRight.hasResetOccurred())
		{
			climberMotorLeft.optimizeBusUtilization();
			climberMotorRight.optimizeBusUtilization();
			climberMotorLeft.getRotorPosition().setUpdateFrequency(20);
			climberMotorRight.getRotorPosition().setUpdateFrequency(20);
		}
    }

    public double getMotorPosition() {

        return climberMotorLeft.getRotorPosition().getValueAsDouble();

    }
 
    public void setTargetPosition(double position) {

        this.position = position;
        climberMotorLeft.setControl(motorControlRequest.withPosition(this.position));

    }
}


