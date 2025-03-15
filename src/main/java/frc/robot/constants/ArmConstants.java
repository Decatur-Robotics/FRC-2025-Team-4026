package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {

    public static final double ZEROING_VOLTAGE = 1.5;

    public static final double STALL_DEBOUNCE_TIME = 0.3;
    public static final double STALL_CURRENT = 50;

    // Stowed position
    public static final double STOWED_POSITION = -3; // -2 is pretty much all the way in

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = -26;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = -3.5;
    public static final double ALGAE_GROUND_INTAKING_POSITION = -18.2;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = -12.4;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = ALGAE_LOW_REEF_INTAKING_POSITION;

    // Scoring positions
    public static final double L1_SCORING_POSITION = -8.25;
    public static final double L2_SCORING_POSITION = -10.5;
    public static final double L2_STAGING_POSITION = -6.5;
    public static final double L3_SCORING_POSITION = -10.5;
    public static final double L3_STAGING_POSITION = -6.5;
    public static final double L4_SCORING_POSITION = -9.5;
    public static final double L4_STAGING_POSITION = -4.5;
    public static final double PROCESSOR_POSITION = -13;
    public static final double NET_POSITION = -4;

    /** The position when the arm is parallel to the floor */
	public static final double LEVEL_POSITION = 0;

    /** The encoder value when the arm is parallel to the floor */
    public static final double THROUGH_BORE_ENCODER_ZERO_OFFSET = 0.152;

    public static final double TALON_ENCODER_TO_ROTATIONS_RATIO = 0.015625; // how many motor encoder rotations it takes to complete one rotation of the mechanism

    public static final double KG = 0.32912; // 0.375

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.2) // 0.2
        .withKI(0)
        .withKD(0)
        .withKS(0.105) // 0.105
        .withKV(0.1376) // 0.1376
        .withKA(0.01); // 0.01
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(50)
        .withMotionMagicAcceleration(80);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

}
