package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Ports;

public class ArmSubsystem extends SubsystemBase {
	
	private TalonFX motorMain;
	
	private double position;

	private MotionMagicVoltage controlRequest;

	private Encoder throughBoreEncoder;

	public ArmSubsystem() {
		motorMain = new TalonFX(Ports.ARM_MOTOR);

		motorMain.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);

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
		
        if(motorMain.hasResetOccurred()) {
			motorMain.optimizeBusUtilization();
			motorMain.getRotorPosition().setUpdateFrequency(20);
		}
	}

	public void setPosition(double position) {
		this.position = position;

		resetTalonEncoder();

		// Arm angle in radians (0 is parallel to the floor)
		double angle = (position - ArmConstants.LEVEL_POSITION) * ArmConstants.TALON_ENCODER_TO_RADIANS_RATIO;

		double gravityFeedForward = Math.cos(angle) * ArmConstants.KG;

		motorMain.setControl(controlRequest.withPosition(position)
				.withFeedForward(gravityFeedForward));
	}

    public double getTalonPosition() {
        return motorMain.getRotorPosition().getValueAsDouble();
    }


	public double getThroughBoreEncoderPosition() {
		return throughBoreEncoder.get();
	}

	public void resetTalonEncoder() {
        double rotations = (getThroughBoreEncoderPosition() - ArmConstants.THROUGH_BORE_ENCODER_ZERO_OFFSET) 
			/ ArmConstants.THROUGH_BORE_ENCODER_TO_TALON_ENCODER_RATIO;
		motorMain.setPosition(rotations);
    }

}

