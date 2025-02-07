package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFXS motorLeft, motorRight;

    private double velocity;
    private double velocityFeedForward;
    private MotionMagicVelocityVoltage motorControlRequest;
    
    public IntakeSubsystem() {
        motorLeft = new TalonFXS(Ports.INTAKE_MOTOR_LEFT);
        motorRight = new TalonFXS(Ports.INTAKE_MOTOR_RIGHT);

        motorRight.setControl(new Follower(motorLeft.getDeviceID(), true));

        TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;

        // Added kS and kV in case they were need for velocity closed-loop control
        motorConfigs.Slot0.kP = IntakeConstants.KP;
        motorConfigs.Slot0.kI = IntakeConstants.KI;
        motorConfigs.Slot0.kD = IntakeConstants.KD;
        motorConfigs.Slot0.kS = IntakeConstants.KS;
        motorConfigs.Slot0.kV = IntakeConstants.KV;

        motorConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motorLeft.getConfigurator().apply(motorConfigs);
        motorRight.getConfigurator().apply(motorConfigs);

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();
        motorLeft.getVelocity().setUpdateFrequency(20);
        
        velocityFeedForward = IntakeConstants.KFF;
        velocity = IntakeConstants.REST_VELOCITY;
        
        motorControlRequest = new MotionMagicVelocityVoltage(velocity);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        motorLeft.setControl(motorControlRequest.withFeedForward(velocityFeedForward));
    }

    public double getVelocity() {
        return motorLeft.getVelocity().getValueAsDouble();
    }

}
