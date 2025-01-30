package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ElevatorConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX motorFollower, motorMain;
    
    private double position;
    
    private MotionMagicDutyCycle motorControlRequest;

    private DigitalInput limitSwitch;
    
    public ElevatorSubsystem() {

        position = ElevatorConstants.INITIAL_POSITION;

        motorFollower = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT);
        motorMain = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT);

        motorFollower.setControl(new Follower(Ports.ELEVATOR_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;

        talonFXConfigs.Slot0.kP = ElevatorConstants.KP; 
        talonFXConfigs.Slot0.kI = ElevatorConstants.KI; 
        talonFXConfigs.Slot0.kD = ElevatorConstants.KD;
        talonFXConfigs.Slot0.kS = ElevatorConstants.KS; 
        talonFXConfigs.Slot0.kV = ElevatorConstants.KV;
        talonFXConfigs.Slot0.kA = ElevatorConstants.KA; 
        talonFXConfigs.Slot0.kG = ElevatorConstants.KG;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        motorMain.getConfigurator().apply(talonFXConfigs);
        motorFollower.getConfigurator().apply(talonFXConfigs);

        motorMain.optimizeBusUtilization();
        motorFollower.optimizeBusUtilization();
        motorMain.getRotorPosition().setUpdateFrequency(20);
        motorFollower.getRotorPosition().setUpdateFrequency(20);

        motorControlRequest = new MotionMagicDutyCycle(position);
        motorMain.setControl(motorControlRequest);
    }

    @Override
    public void periodic() {
        if (motorMain.hasResetOccurred() || motorFollower.hasResetOccurred())
		{
			motorMain.optimizeBusUtilization();
			motorFollower.optimizeBusUtilization();
			motorMain.getRotorPosition().setUpdateFrequency(20);
			motorFollower.getRotorPosition().setUpdateFrequency(20);
		}
    }
 
    public void setPosition(double position) {
        this.position = position;
        motorMain.setControl(motorControlRequest.withPosition(position));
    }

    public double getPosition() {
        return motorMain.getRotorPosition().getValueAsDouble();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

}
