package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ElevatorConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX elevatorMotorRight, elevatorMotorLeft;
    
    private double position;
    private MotionMagicDutyCycle motorControlRequest;
    
    public ElevatorSubsystem() {

        position = ElevatorConstants.ELEVATOR_POSITION_REST;

        elevatorMotorRight = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT);
        elevatorMotorLeft = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT);

        elevatorMotorRight.setControl(new Follower(Ports.ELEVATOR_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = ElevatorConstants.ELEVATOR_MOTOR_KS; 
        slot0Configs.kV = ElevatorConstants.ELEVATOR_MOTOR_KV;
        slot0Configs.kA = ElevatorConstants.ELEVATOR_MOTOR_KA; 
        slot0Configs.kP = ElevatorConstants.ELEVATOR_MOTOR_KP; 
        slot0Configs.kI = ElevatorConstants.ELEVATOR_MOTOR_KI; 
        slot0Configs.kD = ElevatorConstants.ELEVATOR_MOTOR_KD;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;
        talonFXConfigs.MotionMagic.MotionMagicJerk = ElevatorConstants.MOTION_MAGIC_JERK;

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

    public double getMotorPosition() {

        return elevatorMotorLeft.getRotorPosition().getValueAsDouble();

    }
 
    public void setTargetPosition(double position) {

        this.position = position;
        elevatorMotorLeft.setControl(motorControlRequest.withPosition(this.position));

    }
}
