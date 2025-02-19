package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class WristConstants {

    public static final double PARALLEL_CURRENT = 0;
    public static final double PERPENDICULAR_CURRENT = 0;
    public static final double REDUCED_PARALLEL_CURRENT = 0;
    public static final double REDUCED_PERPENDICULAR_CURRENT = 0;

    public static final double MAX_SLAMMED_VELOCITY = 0;

    public static final double SLAM_DEBOUNCE_TIME = 0;

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(0);
    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);

}
