package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    // Robot mass = robot + battery + bumpers
    // Robot mass = 44 kg + 5.85 kg + ???

    public static final RobotConfig CONFIG = new RobotConfig(0, 
        0, 
        new ModuleConfig(0.0508, 4.73, 1, DCMotor.getKrakenX60(1), 120, 1), 
        0.74295);

    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 
        Units.degreesToRadians(0), Units.degreesToRadians(0));

    public static final PIDConstants AUTO_TRANSLATIONAL_CONSTANTS = new PIDConstants(10, 0, 0);
    public static final PIDConstants AUTO_ROTATIONAL_CONSTANTS = new PIDConstants(7, 0, 0);

    public static final PIDController TRANSLATIONAL_CONTROLLER = new PIDController(
        10,
        0,
        0);
    public static final ProfiledPIDController ROTATIONAL_CONTROLLER = new ProfiledPIDController(
        7,
        0,
        0,
        new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI)); // Radians

    /* Translation velocity from SDS, not tuned to robot */
    public static final double MAX_TRANSLATIONAL_VELOCITY = 15.5;
    /* Not tuned */
    public static final double MAX_ROTATIONAL_VELOCITY = 10;

    public static final double TRANSLATIONAL_DEADBAND = MAX_TRANSLATIONAL_VELOCITY * 0.05;
    public static final double ROTATIONAL_DEADBAND = MAX_ROTATIONAL_VELOCITY * 0.05;

    public static final double TRANSLATIONAL_ERROR_MARGIN = 0;
    public static final double ROTATIONAL_ERROR_MARGIN = 0;

}
