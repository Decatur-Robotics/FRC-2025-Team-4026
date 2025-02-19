package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final RobotConfig CONFIG = new RobotConfig(0, 0, null, 0);

    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 
        Units.degreesToRadians(0), Units.degreesToRadians(0));

    public static final PIDConstants TRANSLATIONAL_CONSTANTS = new PIDConstants(0, 0, 0);
    public static final PIDConstants ROTATIONAL_CONSTANTS = new PIDConstants(0, 0, 0);

    public static final PIDController TRANSLATIONAL_CONTROLLER = new PIDController(
        TRANSLATIONAL_CONSTANTS.kP,
        TRANSLATIONAL_CONSTANTS.kI,
        TRANSLATIONAL_CONSTANTS.kD);
    public static final PIDController ROTATIONAL_CONTROLLER = new PIDController(
        ROTATIONAL_CONSTANTS.kP,
        ROTATIONAL_CONSTANTS.kI,
        ROTATIONAL_CONSTANTS.kD);

    /* Translation velocity from SDS, not tuned to robot */
    public static final double MAX_TRANSLATIONAL_VELOCITY = 15.5;
    /* Not tuned */
    public static final double MAX_ROTATIONAL_VELOCITY = 10;

    public static final double TRANSLATIONAL_DEADBAND = MAX_TRANSLATIONAL_VELOCITY * 0.05;
    public static final double ROTATIONAL_DEADBAND = MAX_ROTATIONAL_VELOCITY * 0.05;

    public static final double TRANSLATIONAL_ERROR_MARGIN = 0;
    public static final double ROTATIONAL_ERROR_MARGIN = 0;

}
