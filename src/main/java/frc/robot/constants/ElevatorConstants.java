package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ElevatorConstants {

    public static final double INITIAL_POSITION = SuperstructureConstants.CORAL_STOWED_STATE.elevatorPosition;

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
