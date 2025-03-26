package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {

    public static final double ZEROING_VOLTAGE = 1;

    public static final double STALL_DEBOUNCE_TIME = 0.2;
    public static final double STALL_CURRENT = 40;

    // Stowed position
    public static final double STOWED_POSITION = 0.26; // 0.27 is all the way in

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = -0.07;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = 0.26;
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
            .withMagnetOffset(0.69));

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.0) // 0.3
        .withKI(0)
        .withKD(0)
        .withKS(0.0) // 0.105
        .withKV(0.0) // 0.1376
        .withKA(0.0) // 0.01
        .withKG(0.0) // 0.475
        .withGravityType(GravityTypeValue.Arm_Cosine);
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(50)
        .withMotionMagicAcceleration(100);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Ports.ARM_ENCODER);
        
    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

}
