package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ElevatorConstants {

    // Stowed position
    public static final double STOWED_POSITION = 0;

    // Intaking positions
    public static final double CORAL_GROUND_INTAKING_POSITION = 0;
    public static final double CORAL_HUMAN_PLAYER_INTAKING_POSITION = 10;
    public static final double ALGAE_GROUND_INTAKING_POSITION = 0;
    public static final double ALGAE_LOW_REEF_INTAKING_POSITION = 20.47;
    public static final double ALGAE_HIGH_REEF_INTAKING_POSITION = 36.63;

    // Scoring positions
    public static final double L1_POSITION = 0;
    public static final double MOVE_TO_L2_POSITION = 12.52;
    public static final double PLACE_L2_POSITION = 12.52;
    public static final double DROP_L2_POSITION = 12.52;
    public static final double MOVE_TO_L3_POSITION = 29.18;
    public static final double PLACE_L3_POSITION = 29.18;
    public static final double DROP_L3_POSITION = 29.18;
    public static final double MOVE_TO_L4_POSITION = 54;
    public static final double PLACE_L4_POSITION = 54;
    public static final double DROP_L4_POSITION = 54;
    public static final double PROCESSOR_POSITION = 0;
    public static final double NET_POSITION = 54;

    public static final double ZEROING_VOLTAGE = -2;

    public static final double STALL_DEBOUNCE_TIME = 0.3;
    public static final double STALL_CURRENT = 100;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.5) // 0.5
        .withKI(0)
        .withKD(0)
        .withKS(0.19) // 0.19
        .withKV(0.13) // 0.13
        .withKA(0.007) // 0.007
        .withKG(0.39) // 0.39
        .withGravityType(GravityTypeValue.Elevator_Static);
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(90) 
        .withMotionMagicAcceleration(200); 
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
