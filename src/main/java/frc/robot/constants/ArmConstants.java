package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmConstants {

    public static final double ZEROING_VOLTAGE = 1;

    public static final double STALL_DEBOUNCE_TIME = 0.2;
    public static final double STALL_CURRENT = 40;

    // Stowed position
    public static final double STOWED_POSITION = 0.2; 

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = -0.09;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = 0.2;
    public static final double ALGAE_GROUND_INTAKING_POSITION = 0;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = 0;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = ALGAE_LOW_REEF_INTAKING_POSITION;

    // Scoring positions
    public static final double L1_SCORING_POSITION = 0;
    public static final double L2_SCORING_POSITION = 0;
    public static final double L2_STAGING_POSITION = 0;
    public static final double L3_SCORING_POSITION = 0;
    public static final double L3_STAGING_POSITION = 0;
    public static final double L4_SCORING_POSITION = 0;
    public static final double L4_STAGING_POSITION = 0;
    public static final double PROCESSOR_POSITION = 0;
    public static final double NET_POSITION = 0;

    /** The position when the arm is parallel to the floor */
	public static final double LEVEL_POSITION = 0;

    /** The encoder value when the arm is parallel to the floor */
    public static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0.67));
            // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(22) // 27
        .withKI(0)
        .withKD(0)
        .withKS(0.081) // 0.081
        .withKV(8.15) // 8.55
        .withKA(0)
        .withKG(0.428) // 0.428
        .withGravityType(GravityTypeValue.Arm_Cosine);
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0.6) // 0.6
        .withMotionMagicAcceleration(2); // 2?
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Ports.ARM_ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
        
    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

}
