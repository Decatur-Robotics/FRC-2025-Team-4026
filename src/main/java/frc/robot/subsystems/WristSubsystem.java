package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    private TalonFX motor;

    private double position;

    private MotionMagicDutyCycle controlRequest;

    private Encoder throughBoreEncoder;
    private double offset;

    public WristSubsystem() {
        
        motor = new TalonFX(Ports.WRIST_MOTOR);

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigs.Slot0.kP = WristConstants.KP;
        motorConfigs.Slot0.kI = WristConstants.KI;
        motorConfigs.Slot0.kD = WristConstants.KD;
        motorConfigs.Slot0.kS = WristConstants.KS;
        motorConfigs.Slot0.kV = WristConstants.KV;
        motorConfigs.Slot0.kA = WristConstants.KA;

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CRUISE_VELOCITY;
        motorConfigs.MotionMagic.MotionMagicAcceleration = WristConstants.ACCELERATION;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfigs);

        motor.optimizeBusUtilization();
        motor.getRotorPosition().setUpdateFrequency(20);

        position = WristConstants.INITIAL_POSITION; 

        motor.setControl(controlRequest.withPosition(position));

        // k4X is quadrature encoding
        throughBoreEncoder = new Encoder(Ports.WRIST_ENCODER_A, Ports.WRIST_ENCODER_B, false, Encoder.EncodingType.k4X);

        resetEncoderOffset();
    }

    @Override
    public void periodic() {
        if(motor.hasResetOccurred()){
            motor.optimizeBusUtilization();
            motor.getRotorPosition().setUpdateFrequency(20);
        }
    }

    public void setPosition(double position) {
        this.position = position;

        motor.setControl(controlRequest.withPosition(position + offset));
    }

    public double getRawTalonPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }
    
    public double getThroughBoreEncoderValue() {
        return throughBoreEncoder.get();
    }

    public double getOffsetTalonPosition() {
        return motor.getRotorPosition().getValueAsDouble() - offset;
    }

    public void resetEncoderOffset() {
        double encoderRotations = (getThroughBoreEncoderValue() - WristConstants.ENCODER_ZERO_OFFSET) / WristConstants.ENCODER_COUNTS_PER_REVOLUTION;
        offset = encoderRotations - motor.getRotorPosition().getValueAsDouble();
    }

    // Commands

	public Command resetWristOffset() {
		return runOnce(() -> resetEncoderOffset());
	}

}
