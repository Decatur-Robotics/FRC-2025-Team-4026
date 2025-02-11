package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.ClimberConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX climberMotorRight, climberMotorLeft;
    
    private double position;
    private MotionMagicDutyCycle controlRequest;
    
    public ClimberSubsystem() {
        climberMotorRight = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT);
        climberMotorLeft = new TalonFX(Ports.CLIMBER_MOTOR_LEFT);

        climberMotorRight.setControl(new Follower(Ports.CLIMBER_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // Current limit configs
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;

        // PID configs
        talonFXConfigs.Slot0.kP = ClimberConstants.KP; 
        talonFXConfigs.Slot0.kI = ClimberConstants.KI; 
        talonFXConfigs.Slot0.kD = ClimberConstants.KD;
        talonFXConfigs.Slot0.kS = ClimberConstants.KS; 
        talonFXConfigs.Slot0.kV = ClimberConstants.KV;
        talonFXConfigs.Slot0.kA = ClimberConstants.KA; 

        // Motion Magic configs
        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ClimberConstants.ACCELERATION;

        // Applying configs to each motor
        climberMotorLeft.getConfigurator().apply(talonFXConfigs);
        climberMotorRight.getConfigurator().apply(talonFXConfigs);

        // CAN optimization
        climberMotorLeft.optimizeBusUtilization();
        climberMotorRight.optimizeBusUtilization();
        climberMotorLeft.getRotorPosition().setUpdateFrequency(20);
        
        position = ClimberConstants.INITIAL_POSITION;

        controlRequest = new MotionMagicDutyCycle(position);
        climberMotorLeft.setControl(controlRequest);
    }

    
    @Override
    public void periodic() {
        if (climberMotorLeft.hasResetOccurred() || climberMotorRight.hasResetOccurred()) {
			climberMotorLeft.optimizeBusUtilization();
			climberMotorRight.optimizeBusUtilization();
			climberMotorLeft.getRotorPosition().setUpdateFrequency(20);
		}
    }
    
    /**
     * Sets the position of the climber motor based on a target position
     * @param position
     */
    public void setPosition(double position) {
        this.position = position;
        climberMotorLeft.setControl(controlRequest.withPosition(this.position));
    }

    /**
     * @return Position of the motor in mechanism rotations
     */
    public double getPosition() {
        return climberMotorLeft.getRotorPosition().getValueAsDouble();
    }

    /**
     * Moves the climber to an upright position, waits for an interrupt, then moves back down
     */
    public Command climbCommand() {
        return startEnd(
            () -> this.setPosition(ClimberConstants.CLIMBED_POSITION),
            () -> this.setPosition(ClimberConstants.INITIAL_POSITION));
    }

}


