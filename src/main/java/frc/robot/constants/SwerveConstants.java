package frc.robot.constants;

import static edu.wpi.first.units.Units.Kilograms;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    // Robot mass = robot + battery + bumpers
    // Robot mass = 52.16 kg + 5.85 kg + 5.126 kg = 63.136 kg

    public static final RobotConfig CONFIG = new RobotConfig(63.136, 
        5.8082, 
        new ModuleConfig(0.05, 4.67, 1.2, DCMotor.getKrakenX60(1).withReduction(6.75), 68, 1), 
        0.74295);

    public static final PathConstraints CONSTRAINTS = new PathConstraints(0, 0, 
        Units.degreesToRadians(0), Units.degreesToRadians(0));

    public static final PIDConstants AUTO_TRANSLATIONAL_CONSTANTS = new PIDConstants(10, 0, 0);
    public static final PIDConstants AUTO_ROTATIONAL_CONSTANTS = new PIDConstants(7, 0, 0);

    public static final PIDController TRANSLATIONAL_CONTROLLER = new PIDController(
        1,
        0,
        0);
    public static final PIDController ROTATIONAL_CONTROLLER = new PIDController(
        1,
        0,
        0); // Radians

    /* Translation velocity from SDS, not tuned to robot */
    public static final double MAX_TRANSLATIONAL_VELOCITY = 4.73;
    /* Not tuned */
    public static final double MAX_ROTATIONAL_VELOCITY = 10.76;

    public static final double TRANSLATIONAL_DRIVER_DEADBAND = MAX_TRANSLATIONAL_VELOCITY * 0.05;
    public static final double ROTATIONAL_DRIVER_DEADBAND = MAX_ROTATIONAL_VELOCITY * 0.05;

    public static final double TRANSLATIONAL_AUTO_DEADBAND = 0; // MAX_TRANSLATIONAL_VELOCITY * 0.01;
    public static final double ROTATIONAL_AUTO_DEADBAND = 0; // MAX_ROTATIONAL_VELOCITY * 0.01;

    public static final double TRANSLATIONAL_ALIGNED_ERROR_MARGIN = 0.025;
    public static final double ROTATIONAL_ALIGNED_ERROR_MARGIN = 0.03;
    public static final double VELOCITY_ALIGNED_ERROR_MARGIN = 0.1;

    
    public static final double TRANSLATIONAL_NEAR_ALIGNED_ERROR_MARGIN = 0.75;
    public static final double ROTATIONAL_NEAR_ALIGNED_ERROR_MARGIN = 0.25;
    public static final double VELOCITY_NEAR_ALIGNED_ERROR_MARGIN = 1.5;

    public static final double PRECISION_MODE_SCALAR = 0.25;

    public static final double MAX_DRIVE_TO_POSE_SPEED = 4;

}
