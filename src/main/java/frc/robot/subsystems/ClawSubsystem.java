package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    
    private TalonFX motor;

    private double position;

    private MotionMagicDutyCycle controlRequest;

    private Encoder throughBoreEncoder;
    private double offset;

    public ClawSubsystem() {
        motor = new TalonFX(Ports.CLAW_MOTOR); 

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = ClawConstants.STATOR_CURRENT_LIMIT;
     
        motorConfigs.Slot0.kP = ClawConstants.KP; 
        motorConfigs.Slot0.kI = ClawConstants.KI; 
        motorConfigs.Slot0.kD = ClawConstants.KD;
        motorConfigs.Slot0.kS = ClawConstants.KS; 
        motorConfigs.Slot0.kV = ClawConstants.KV;
        motorConfigs.Slot0.kA = ClawConstants.KA; 
        motorConfigs.Slot0.kG = ClawConstants.KG;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = ClawConstants.CRUISE_VELOCITY;
        motorConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.ACCELERATION;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfigs);

        motor.optimizeBusUtilization();
        motor.getRotorPosition().setUpdateFrequency(20);

        position = ClawConstants.CORAL_POSITION;

        controlRequest = new MotionMagicDutyCycle(position);
        motor.setControl(controlRequest);

        // k4X is quadrature encoding
        throughBoreEncoder = new Encoder(Ports.CLAW_ENCODER_A, Ports.CLAW_ENCODER_B, false, Encoder.EncodingType.k4X);

        resetEncoderOffset();
    }

    @Override
    public void periodic() {
        if (motor.hasResetOccurred())
		{
			motor.optimizeBusUtilization();
			motor.getRotorPosition().setUpdateFrequency(20);
		}
    }

    public void setPosition(double position) {
        this.position = position;
        motor.setControl(controlRequest.withPosition(position + offset));
    }

    public double getRawTalonPosition() {
        return (motor.getRotorPosition().getValueAsDouble());
    }  

    public double getThroughBoreEncoderValue() {
        return throughBoreEncoder.get();
    }

    public double getOffsetTalonPosition() {
        return motor.getRotorPosition().getValueAsDouble() - offset;
    }

    public void resetEncoderOffset() {
        double rotations = (getThroughBoreEncoderValue() - ClawConstants.ENCODER_ZERO_OFFSET) / ClawConstants.ENCODER_COUNTS_PER_REVOLUTION;
        offset = rotations - motor.getRotorPosition().getValueAsDouble();
    }

    // Commands

	public Command resetClawOffset() {
		return runOnce(() -> resetEncoderOffset());
	}

}
