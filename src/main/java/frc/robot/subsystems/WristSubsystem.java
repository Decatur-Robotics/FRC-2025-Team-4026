package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    private TalonFX motor;

    private double position;

    private MotionMagicDutyCycle motorControlRequest;

    private Encoder encoder;

    public WristSubsystem() {
        
        motor = new TalonFX(Ports.WRIST_MOTOR);

        TalonFXConfiguration talonFXconfigs = new TalonFXConfiguration();

        talonFXconfigs.Slot0.kP = WristConstants.KP;
        talonFXconfigs.Slot0.kI = WristConstants.KI;
        talonFXconfigs.Slot0.kD = WristConstants.KD;
        talonFXconfigs.Slot0.kS = WristConstants.KS;
        talonFXconfigs.Slot0.kV = WristConstants.KV;
        talonFXconfigs.Slot0.kA = WristConstants.KA;

        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;

        talonFXconfigs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CRUISE_VELOCITY;
        talonFXconfigs.MotionMagic.MotionMagicAcceleration = WristConstants.ACCELERATION;

        motor.getConfigurator().apply(talonFXconfigs);

        motor.optimizeBusUtilization();
        motor.getRotorPosition().setUpdateFrequency(20);

        position = WristConstants.INITIAL_POSITION; 

        motor.setControl(motorControlRequest.withPosition(position));

        //k4X is quadrature encoding
        encoder = new Encoder(Ports.WRIST_ENCODER_A, Ports.WRIST_ENCODER_B, false, Encoder.EncodingType.k4X);
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

        motor.setControl(motorControlRequest);
    }

    public double getPosition() {
        return motor.getRotorPosition().getValueAsDouble();
    }
    
    public double getEncoderValue() {
        return encoder.get();
    }

    public void resetEncoder() {
        double rawEncoderValue = getEncoderValue();
        double rotations = rawEncoderValue / (double) WristConstants.kEncoderCountsPerRevolution;
        motor.setPosition(rotations);
    }
}
