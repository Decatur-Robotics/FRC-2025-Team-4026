package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathSetpoints {

    // Coral scoring locations
    public static Pose2d REEF_A = new Pose2d(1, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_B = new Pose2d(0, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_C = new Pose2d(2, 10, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_D = new Pose2d(5, 2, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_E = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_F = new Pose2d(5, 10, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_G = new Pose2d(10, 0, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_H = new Pose2d(10, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_I = new Pose2d(10, 10, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_J = new Pose2d(15, 0, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_K = new Pose2d(15, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_L = new Pose2d(15, 10, Rotation2d.fromDegrees(0));

    // Algae pick up locations
    public static Pose2d REEF_AB = new Pose2d(1, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_CD = new Pose2d(2, 10, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_EF = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_GH = new Pose2d(10, 0, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_IJ = new Pose2d(10, 10, Rotation2d.fromDegrees(0));
    public static Pose2d REEF_KL = new Pose2d(15, 5, Rotation2d.fromDegrees(0));

    // Algae scoring locations
    public static Pose2d PROCESSOR = new Pose2d(20, 20, Rotation2d.fromDegrees(0));
    // Would be cool to get net pathing so that it locks the x axis and rotation but lets us control the y axis
    public static Pose2d NET = new Pose2d(20, 20, Rotation2d.fromDegrees(0));
}
