package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.WristConstants;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{
    
    private TalonFX motor;

    private double position;
    private MotionMagicDutyCycle controlRequest;

    private Encoder encoder;

    public ClawSubsystem() {
        motor = new TalonFX(Ports.CLAW_MOTOR); 

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ClawConstants.STATOR_CURRENT_LIMIT;
     
        talonFXConfigs.Slot0.kP = ClawConstants.KP; 
        talonFXConfigs.Slot0.kI = ClawConstants.KI; 
        talonFXConfigs.Slot0.kD = ClawConstants.KD;
        talonFXConfigs.Slot0.kS = ClawConstants.KS; 
        talonFXConfigs.Slot0.kV = ClawConstants.KV;
        talonFXConfigs.Slot0.kA = ClawConstants.KA; 
        talonFXConfigs.Slot0.kG = ClawConstants.KG;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ClawConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ClawConstants.ACCELERATION;

        motor.getConfigurator().apply(talonFXConfigs);

        motor.optimizeBusUtilization();
        motor.getRotorPosition().setUpdateFrequency(20);

        position = ClawConstants.CORAL_POSITION;

        controlRequest = new MotionMagicDutyCycle(position);
        motor.setControl(controlRequest);

        //k4X is quadrature encoding
        encoder = new Encoder(Ports.CLAW_ENCODER_A, Ports.CLAW_ENCODER_B, false, Encoder.EncodingType.k4X);
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
        motor.setControl(controlRequest);
    }

    public double getPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }  

    public double getEncoderValue() {
        return encoder.get();
    }

    public void resetEncoder() {
        double rawEncoderValue = getEncoderValue();
        double rotations = rawEncoderValue / (double) ClawConstants.K_ENCODER_COUNTS_PER_REVOLUTION;
        motor.setPosition(rotations);
    }

}
