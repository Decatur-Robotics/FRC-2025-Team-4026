package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {

	public static final double KG = 0;

    public static final double TALON_ENCODER_TO_RADIANS_RATIO = 0;

    /** The position when the arm is parallel to the floor */
	public static final double LEVEL_POSITION = 0;

    public static final double INITIAL_POSITION = SuperstructureConstants.CORAL_STOWED_STATE.armPosition;

    /** The encoder value when the arm is parallel to the floor */
    public static final int THROUGH_BORE_ENCODER_ZERO_OFFSET = 0;
    public static final int THROUGH_BORE_ENCODER_TO_TALON_ENCODER_RATIO = 8192; // how many encoder ticks it takes to complete one rotation of the mechanism for this specific encoder
    
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(0);
    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0)
        .withKA(0);
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

}
