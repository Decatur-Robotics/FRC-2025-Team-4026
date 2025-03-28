package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

    public static final double CORAL_REST_VELOCITY = 0;
    public static final double ALGAE_REST_VELOCITY = 120;
    public static final double CORAL_INTAKE_VELOCITY = 120;
    public static final double ALGAE_INTAKE_VELOCITY = 120;
    public static final double ALGAE_REMOVE_VELOCITY = -80;
    public static final double L1_EJECT_VELOCITY = -20;
    public static final double BRANCH_EJECT_VELOCITY = -15;
    public static final double PROCESSOR_EJECT_VELOCITY = -120;
    public static final double NET_EJECT_VELOCITY = -120;

    public static final double CORAL_STALL_DEBOUNCE_TIME = 0.1;
    public static final int CORAL_STALL_CURRENT = 50;
    
    public static final double ALGAE_STALL_DEBOUNCE_TIME = 0.5;
    public static final int ALGAE_STALL_CURRENT = 55;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(60);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.05) // 0.05
        .withKI(0)
        .withKD(0)
        .withKS(0.349) // 0.349
        .withKV(0.1) // 0.1
        .withKA(0);
    public static final MotorOutputConfigs LEFT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    public static final MotorOutputConfigs RIGHT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final TalonFXConfiguration MOTOR_LEFT_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIGS);
    public static final TalonFXConfiguration MOTOR_RIGHT_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withMotorOutput(RIGHT_MOTOR_OUTPUT_CONFIGS);

}
