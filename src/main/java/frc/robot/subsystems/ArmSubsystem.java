package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Ports;

public class ArmSubsystem extends SubsystemBase {
	private TalonFX armMotorRight, armMotorLeft;
	
	private double position;

	private MotionMagicDutyCycle motorControlRequest;

	public ArmSubsystem()
	{
		armMotorRight = new TalonFX(Ports.ARM_RIGHT_MOTOR);
		armMotorLeft = new TalonFX(Ports.ARM_LEFT_MOTOR);

		armMotorRight.setControl(new Follower(armMotorLeft.getDeviceID(), true));

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
        
		armMotorRight.getConfigurator().apply(mainMotorConfigs);
		armMotorLeft.getConfigurator().apply(mainMotorConfigs);

		position = ArmConstants.INITIAL_POSITION;

		RobotContainer.getShuffleboardTab().add("Actual Arm Mount Rotation", getPosition());
		RobotContainer.getShuffleboardTab().add("Target Arm Mount Rotation", position);

        if(armMotorRight.hasResetOccurred()||armMotorLeft.hasResetOccurred()){
			armMotorRight.optimizeBusUtilization();
			armMotorLeft.optimizeBusUtilization();
			
			armMotorLeft.getRotorPosition().setUpdateFrequency(20);
		}
	}

	public void setPosition(double position)
	{
		this.position = position;

		// Arm angle in radians (0 is parallel to the floor)
		double angle = (position - ArmConstants.LEVEL_POSITION) * ArmConstants.ENCODER_TO_RADIANS_FACTOR;

		double gravityFeedForward = Math.cos(angle) * ArmConstants.KG;

		armMotorLeft.setControl(motorControlRequest.withPosition(position)
				.withFeedForward(gravityFeedForward));
	}

    public double getPosition() {
        return armMotorRight.getRotorPosition().getValueAsDouble();
    }
}

