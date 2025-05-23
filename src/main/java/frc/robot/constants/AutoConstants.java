package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {

    public static final Pose2d BLUE_LEFT_INITIAL_POSE = new Pose2d(7.23, 5.33, Rotation2d.kCCW_90deg); // y: 7.53
    public static final Pose2d BLUE_CENTER_INITIAL_POSE = new Pose2d(7.23, 4.026, Rotation2d.kZero);
    public static final Pose2d BLUE_RIGHT_INITIAL_POSE = new Pose2d(7.23, 2.73, Rotation2d.kCW_90deg); // y: 0.52

    public static final Pose2d RED_LEFT_INITIAL_POSE = new Pose2d(10.31, 2.73, Rotation2d.kCW_90deg); // y: 0.52
    public static final Pose2d RED_CENTER_INITIAL_POSE = new Pose2d(10.31, 4.026, Rotation2d.k180deg);
    public static final Pose2d RED_RIGHT_INITIAL_POSE = new Pose2d(10.31, 5.33, Rotation2d.kCCW_90deg); // y: 7.53

    public static final double Y_TOLERANCE = 1.6;

}
