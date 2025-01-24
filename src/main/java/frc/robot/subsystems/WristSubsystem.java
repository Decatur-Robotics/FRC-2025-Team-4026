package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private TalonFX wristMotor;
    private double position;
    private MotionMagicDutyCycle motorControlRequest;
    public WristSubsystem() {
        wristMotor = new TalonFX(Ports.WRIST_MOTOR);

        TalonFXConfiguration talonFXconfigs = new TalonFXConfiguration();

        talonFXconfigs.Slot0.kP = WristConstants.KP;
        talonFXconfigs.Slot0.kI = WristConstants.KI;
        talonFXconfigs.Slot0.kD = WristConstants.KD;
        talonFXconfigs.Slot0.kS = WristConstants.KS;
        talonFXconfigs.Slot0.kV = WristConstants.KV;
        talonFXconfigs.Slot0.kA = WristConstants.KA;

        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = WristConstants.WRIST_STATOR_CURRENT_LIMIT;

        talonFXconfigs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        talonFXconfigs.MotionMagic.MotionMagicAcceleration = WristConstants.MOTION_MAGIC_ACCELERATION;

        wristMotor.getConfigurator().apply(talonFXconfigs);

        wristMotor.optimizeBusUtilization();
        wristMotor.getRotorPosition().setUpdateFrequency(20);

       position = WristConstants.INITIAL_POSITION; 

    }

    @Override
    public void periodic() {
        if(wristMotor.hasResetOccurred()){
            wristMotor.optimizeBusUtilization();
            wristMotor.getRotorPosition().setUpdateFrequency(20);
        }
    }

    public void setWristPosition(double position){
        wristMotor.setControl(motorControlRequest);
    }

    public double getWristPosition(){
        return wristMotor.getRotorPosition().getValueAsDouble();
    }
    
}
