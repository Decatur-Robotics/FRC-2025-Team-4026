package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Ports;
import frc.robot.constants.WristConstants;

public class ArmSubsystem extends SubsystemBase {
	
	private TalonFX motorFollower, motorMain;
	
	private double position;

	private MotionMagicDutyCycle motorControlRequest;

	private Encoder encoder;

	public ArmSubsystem() {
		motorFollower = new TalonFX(Ports.ARM_MOTOR_RIGHT);
		motorMain = new TalonFX(Ports.ARM_MOTOR_LEFT);

		motorFollower.setControl(new Follower(motorMain.getDeviceID(), true));

		TalonFXConfiguration mainMotorConfigs = new TalonFXConfiguration();

		mainMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		mainMotorConfigs.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;

		mainMotorConfigs.Slot0.kP = ArmConstants.KP;
		mainMotorConfigs.Slot0.kI = ArmConstants.KI;
		mainMotorConfigs.Slot0.kD = ArmConstants.KD;
		mainMotorConfigs.Slot0.kS = ArmConstants.KS;
		mainMotorConfigs.Slot0.kV = ArmConstants.KV;
		mainMotorConfigs.Slot0.kA = ArmConstants.KA;

        mainMotorConfigs.MotionMagic.MotionMagicAcceleration = ArmConstants.ACCELERATION;
        mainMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.CRUISE_VELOCITY;
        
		motorFollower.getConfigurator().apply(mainMotorConfigs);
		motorMain.getConfigurator().apply(mainMotorConfigs);

        motorFollower.optimizeBusUtilization();
		motorMain.optimizeBusUtilization();
		motorMain.getRotorPosition().setUpdateFrequency(20);

		position = ArmConstants.INITIAL_POSITION;

		motorControlRequest = new MotionMagicDutyCycle(position);

		//k4X is quadrature encoding
		encoder = new Encoder(Ports.ARM_ENCODER_A, Ports.ARM_ENCODER_B, false, Encoder.EncodingType.k4X);

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

		// Arm angle in radians (0 is parallel to the floor)
		double angle = (position - ArmConstants.LEVEL_POSITION) * ArmConstants.ENCODER_TO_RADIANS_FACTOR;

		double gravityFeedForward = Math.cos(angle) * ArmConstants.KG;

		motorMain.setControl(motorControlRequest.withPosition(position)
				.withFeedForward(gravityFeedForward));
	}

    public double getPosition() {
        return motorFollower.getRotorPosition().getValueAsDouble();
    }


	public double getEncoderValue() {
		return encoder.get();
	}

	public void resetEncoder() {
        double rawEncoderValue = getEncoderValue();
        double rotations = rawEncoderValue / (double) ArmConstants.kEncoderCountsPerRevolution;
        motorMain.setPosition(rotations);
    }

}

