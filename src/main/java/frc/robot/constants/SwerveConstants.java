package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final RobotConfig CONFIG = new RobotConfig(0, 0, null, 0);

    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 
                                                                          Units.degreesToRadians(0), Units.degreesToRadians(0));

    public static final PIDController TRANSLATIONAL_CONTROLLER = new PIDController(0, 0, 0);
    public static final PIDController ANGULAR_CONTROLLER = new PIDController(0, 0, 0);

    /* Translation velocity from SDS, not tuned to robot */
    public static final double MAX_TRANSLATION_VELOCITY = 15.5;
    /* Not tuned */
    public static final double MAX_ANGULAR_VELOCITY = 10;

    public static final double TRANSLATIONAL_DEADBAND = MAX_TRANSLATION_VELOCITY * 0.05;
    public static final double ANGULAR_DEADBAND = MAX_ANGULAR_VELOCITY * 0.05;

}
