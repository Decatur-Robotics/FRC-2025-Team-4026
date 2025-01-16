package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ElevatorConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX elevatorMotorRight, elevatorMotorLeft;
    
    private double position;
    private MotionMagicDutyCycle motorControlRequest;
    
    public ElevatorSubsystem() {

        position = ElevatorConstants.INITIAL_POSITION;

        elevatorMotorRight = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT);
        elevatorMotorLeft = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT);

        elevatorMotorRight.setControl(new Follower(Ports.ELEVATOR_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;

        talonFXConfigs.Slot0.kP = ElevatorConstants.KP; 
        talonFXConfigs.Slot0.kI = ElevatorConstants.KI; 
        talonFXConfigs.Slot0.kD = ElevatorConstants.KD;
        talonFXConfigs.Slot0.kS = ElevatorConstants.KS; 
        talonFXConfigs.Slot0.kV = ElevatorConstants.KV;
        talonFXConfigs.Slot0.kA = ElevatorConstants.KA; 
        talonFXConfigs.Slot0.kG = ElevatorConstants.KG;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        elevatorMotorLeft.getConfigurator().apply(talonFXConfigs);
        elevatorMotorRight.getConfigurator().apply(talonFXConfigs);

        elevatorMotorLeft.optimizeBusUtilization();
        elevatorMotorRight.optimizeBusUtilization();
        elevatorMotorLeft.getRotorPosition().setUpdateFrequency(20);
        elevatorMotorRight.getRotorPosition().setUpdateFrequency(20);

        motorControlRequest = new MotionMagicDutyCycle(position);
        elevatorMotorLeft.setControl(motorControlRequest);
    }

    @Override
    public void periodic() {
        if (elevatorMotorLeft.hasResetOccurred() || elevatorMotorRight.hasResetOccurred())
		{
			elevatorMotorLeft.optimizeBusUtilization();
			elevatorMotorRight.optimizeBusUtilization();
			elevatorMotorLeft.getRotorPosition().setUpdateFrequency(20);
			elevatorMotorRight.getRotorPosition().setUpdateFrequency(20);
		}
    }
 
    public void setPosition(double position) {
        this.position = position;
        elevatorMotorLeft.setControl(motorControlRequest.withPosition(position));
    }

    public double getPosition() {
        return elevatorMotorLeft.getRotorPosition().getValueAsDouble();
    }

}
