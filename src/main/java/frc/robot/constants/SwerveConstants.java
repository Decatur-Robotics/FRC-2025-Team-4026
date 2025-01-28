package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final RobotConfig CONFIG = new RobotConfig(0, 0, null, 0);

    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 
                                                                          Units.degreesToRadians(0), Units.degreesToRadians(0));
    
    public static final double MAX_ROTATION_VELOCITY = 10;

}
