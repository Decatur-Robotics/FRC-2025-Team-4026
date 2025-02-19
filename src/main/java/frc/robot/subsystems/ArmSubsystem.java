package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Ports;

public class ArmSubsystem extends SubsystemBase {
	
	private TalonFX motorFollower, motorMain;
	
	private double position;

	private MotionMagicVoltage controlRequest;

	private Encoder throughBoreEncoder;

	public ArmSubsystem() {
		motorFollower = new TalonFX(Ports.ARM_MOTOR_RIGHT);
		motorMain = new TalonFX(Ports.ARM_MOTOR_LEFT);

		motorFollower.setControl(new Follower(motorMain.getDeviceID(), true));

		TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

		motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfigs.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;

		motorConfigs.Slot0.kP = ArmConstants.KP;
		motorConfigs.Slot0.kI = ArmConstants.KI;
		motorConfigs.Slot0.kD = ArmConstants.KD;
		motorConfigs.Slot0.kS = ArmConstants.KS;
		motorConfigs.Slot0.kV = ArmConstants.KV;
		motorConfigs.Slot0.kA = ArmConstants.KA;

        motorConfigs.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION;
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
		motorFollower.getConfigurator().apply(motorConfigs);
		motorMain.getConfigurator().apply(motorConfigs);

        motorFollower.optimizeBusUtilization();
		motorMain.optimizeBusUtilization();
		motorMain.getRotorPosition().setUpdateFrequency(20);

		position = ArmConstants.INITIAL_POSITION;

		controlRequest = new MotionMagicVoltage(position);

		// k4X is quadrature encoding
		throughBoreEncoder = new Encoder(Ports.ARM_ENCODER_A, Ports.ARM_ENCODER_B, false, Encoder.EncodingType.k4X);

		resetTalonEncoder();
	}
	
	@Override
	public void periodic() {
		
        if(motorFollower.hasResetOccurred()||motorMain.hasResetOccurred()){

			motorFollower.optimizeBusUtilization();
			motorMain.optimizeBusUtilization();
			motorMain.getRotorPosition().setUpdateFrequency(20);
		}
	}

	public void setPosition(double position) {
		this.position = position;

		resetTalonEncoder();

		// Arm angle in radians (0 is parallel to the floor)
		double angle = (position - ArmConstants.LEVEL_POSITION) * ArmConstants.ENCODER_TO_RADIANS_FACTOR;

		double gravityFeedForward = Math.cos(angle) * ArmConstants.KG;

		motorMain.setControl(controlRequest.withPosition(position)
				.withFeedForward(gravityFeedForward));
	}

    public double getTalonPosition() {
        return motorFollower.getRotorPosition().getValueAsDouble();
    }


	public double getThroughBoreEncoderPosition() {
		return throughBoreEncoder.get();
	}

	public void resetTalonEncoder() {
        double rotations = (getThroughBoreEncoderPosition() - ArmConstants.THROUGH_BORE_ENCODER_ZERO_OFFSET) 
			/ ArmConstants.THROUGH_BORE_ENCODER_TO_TALON_ENCODER_RATIO;
		motorMain.setPosition(rotations);
    }

	// Commands

	public Command resetArmTalonEncoder() {
		return runOnce(() -> resetTalonEncoder());
	}

}

