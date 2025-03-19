package frc.robot.constants;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

    public static final double CORAL_REST_VELOCITY = 0;
    public static final double ALGAE_REST_VELOCITY = 25;
    public static final double INTAKE_VELOCITY = 200;
    public static final double L1_EJECT_VELOCITY = -40;
    public static final double BRANCH_EJECT_VELOCITY = -40;
    public static final double PROCESSOR_EJECT_VELOCITY = -200;
    public static final double NET_EJECT_VELOCITY = -200;

    public static final double STALL_DEBOUNCE_TIME = 0.2;
    public static final int STALL_CURRENT = 20;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(30);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0.05) // 0.05
        .withKI(0)
        .withKD(0)
        .withKS(0.23) // 0.23
        .withKV(0.0658) // 0.0658
        .withKA(0.002); // 0.002
    public static final CommutationConfigs COMMUTATION_CONFIGS = new CommutationConfigs()
        .withMotorArrangement(MotorArrangementValue.NEO550_JST);
    public static final MotorOutputConfigs LEFT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    public static final MotorOutputConfigs RIGHT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final TalonFXSConfiguration MOTOR_LEFT_CONFIG = new TalonFXSConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withCommutation(COMMUTATION_CONFIGS)
        .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIGS);
    public static final TalonFXSConfiguration MOTOR_RIGHT_CONFIG = new TalonFXSConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withSlot0(SLOT_0_CONFIGS)
        .withCommutation(COMMUTATION_CONFIGS)
        .withMotorOutput(RIGHT_MOTOR_OUTPUT_CONFIGS);

}
