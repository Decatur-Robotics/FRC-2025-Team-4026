package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.ElevatorConstants;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;


public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX climberMotorRight, climberMotorLeft;
    
        private double elevatorPosition = ElevatorConstants.ELEVATOR_POSITION_REST;
        private MotionMagicDutyCycle motorControlRequest;
    
        public ElevatorSubsystem() {
            this.climberMotorRight = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT);
            this.climberMotorLeft = new TalonFX(Ports.CLIMBER_MOTOR_LEFT);

        this.climberMotorRight.setControl(new Follower(Ports.CLIMBER_MOTOR_LEFT, true));

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;

        slot0Configs.kS = ElevatorConstants.ELEVATOR_MOTOR_KS; 
        slot0Configs.kV = ElevatorConstants.ELEVATOR_MOTOR_KV;
        slot0Configs.kA = ElevatorConstants.ELEVATOR_MOTOR_KA; 
        slot0Configs.kP = ElevatorConstants.ELEVATOR_MOTOR_KP; 
        slot0Configs.kI = ElevatorConstants.ELEVATOR_MOTOR_KI; 
        slot0Configs.kD = ElevatorConstants.ELEVATOR_MOTOR_KD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY; 
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION; 
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MOTION_MAGIC_JERK;

        climberMotorLeft.getConfigurator().apply(talonFXConfigs);

        motorControlRequest = new MotionMagicDutyCycle(elevatorPosition);
    }

    public double getMotorPosition() {

        return climberMotorLeft.getRotorPosition().getValueAsDouble();

    }
 
    public void setTargetPosition(double position) {

        this.elevatorPosition = position;
        climberMotorLeft.setControl(motorControlRequest.withPosition(this.elevatorPosition));

    }
}
