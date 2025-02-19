package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ElevatorConstants {

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = 0;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = 0;
    public static final double ALGAE_GROUND_INTAKING_POSITION = 0;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = 0;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = 0;

    // Stowed polsition
    public static final double STOWED_POSITION = 0;

    // Scoring positions
    public static final double L1_POSITION = 0;
    public static final double MOVE_TO_L2_POSITION = 0;
    public static final double SCORE_L2_POSITION = 0;
    public static final double MOVE_TO_L3_POSITION = 0;
    public static final double SCORE_L3_POSITION = 0;
    public static final double MOVE_TO_L4_POSITION = 0;
    public static final double SCORE_L4_POSITION = 0;
    public static final double PROCESSOR_POSITION = 0;
    public static final double NET_POSITION = 0;

    public static final double ZEROING_VOLTAGE = 0;

    public static final double STALL_DEBOUNCE_TIME = 0;
    public static final double STALL_CURRENT = 0;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(0);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKG(0)
        .withGravityType(GravityTypeValue.Elevator_Static);
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0)
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

    public static final CurrentLimitsConfigs ZEROING_CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(0);

}
