package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {

    // Stowed position
    public static final double STOWED_POSITION = -0.5;

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = 0;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = -5.5;
    public static final double ALGAE_GROUND_INTAKING_POSITION = 0;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = 0;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = 0;

    // Scoring positions
    public static final double L1_POSITION = 0;
    public static final double MOVE_TO_L2_POSITION = 0;
    public static final double SCORE_L2_POSITION = 0;
    public static final double MOVE_TO_L3_POSITION = 0;
    public static final double SCORE_L3_POSITION = 0;
    public static final double MOVE_TO_L4_POSITION = -0.5;
    public static final double SCORE_L4_POSITION = -5.5;
    public static final double PROCESSOR_POSITION = 0;
    public static final double NET_POSITION = 0;

    /** The position when the arm is parallel to the floor */
	public static final double LEVEL_POSITION = 0;

    /** The encoder value when the arm is parallel to the floor */
    public static final double THROUGH_BORE_ENCODER_ZERO_OFFSET = 0.41;

    public static final double TALON_ENCODER_TO_ROTATIONS_RATIO = 0.015625; // how many motor encoder rotations it takes to complete one rotation of the mechanism

    public static final double KG = 0.375;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.2)
        .withKI(0)
        .withKD(0)
        .withKS(0.105) // 0.105
        .withKV(0.1176) // 0.1176
        .withKA(0.01); // 0.01
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(40)
        .withMotionMagicAcceleration(100);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

}
