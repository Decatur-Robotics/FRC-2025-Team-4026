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

    private TalonFX ClimberMotorRight, ClimberMotorLeft;
    
    private double position;
    private MotionMagicDutyCycle motorControlRequest;
    
    public ClimberSubsystem() {

        position = ClimberConstants.CLIMBER_POSITION_REST;

        ClimberMotorRight = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT);
        ClimberMotorLeft = new TalonFX(Ports.CLIMBER_MOTOR_LEFT);

        ClimberMotorRight.setControl(new Follower(Ports.CLIMBER_MOTOR_LEFT, true));

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

        ClimberMotorLeft.getConfigurator().apply(talonFXConfigs);
        ClimberMotorRight.getConfigurator().apply(talonFXConfigs);

        ClimberMotorLeft.optimizeBusUtilization();
        ClimberMotorRight.optimizeBusUtilization();
        ClimberMotorLeft.getRotorPosition().setUpdateFrequency(20);
        ClimberMotorRight.getRotorPosition().setUpdateFrequency(20);

        motorControlRequest = new MotionMagicDutyCycle(position);
        ClimberMotorLeft.setControl(motorControlRequest);

        
    }

    @Override
    public void periodic() {
        if (ClimberMotorLeft.hasResetOccurred() || ClimberMotorRight.hasResetOccurred())
		{
			ClimberMotorLeft.optimizeBusUtilization();
			ClimberMotorRight.optimizeBusUtilization();
			ClimberMotorLeft.getRotorPosition().setUpdateFrequency(20);
			ClimberMotorRight.getRotorPosition().setUpdateFrequency(20);
		}
    }

    public double getMotorPosition() {

        return ClimberMotorLeft.getRotorPosition().getValueAsDouble();

    }
 
    public void setTargetPosition(double position) {

        this.position = position;
        ClimberMotorLeft.setControl(motorControlRequest.withPosition(this.position));

    }
}


