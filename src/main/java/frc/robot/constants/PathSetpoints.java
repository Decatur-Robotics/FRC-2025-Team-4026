package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathSetpoints {

    // Robot center should 25.125 inches from reef face for coral scoring
    // Robot center should 0.638175 meters from reef face for coral scoring
    // 6.47 inches from apriltag to reef branch on parallel plane
    // 0.164338 meters from apriltag to reef branch on parallel plane

    // Field center
    public static final Translation2d FIELD_CENTER = new Translation2d(8.774176, 4.026281);

    // Coral scoring locations
    public final static Pose2d BLUE_REEF_A = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_B = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_C = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_D = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_E = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_F = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_G = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_H = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_I = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_J = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_K = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_L = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_A = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_B = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_C = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_D = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_E = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_F = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_G = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_H = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_I = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_J = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_K = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_L = new Pose2d(15.5, 6.5, Rotation2d.fromDegrees(0));

    public final static Pose2d[] CORAL_SCORING_POSES = {BLUE_REEF_A, BLUE_REEF_B, BLUE_REEF_C, BLUE_REEF_D, BLUE_REEF_E, BLUE_REEF_F, BLUE_REEF_G, BLUE_REEF_H, BLUE_REEF_I, BLUE_REEF_J, BLUE_REEF_K, BLUE_REEF_L,
        RED_REEF_A, RED_REEF_B, RED_REEF_C, RED_REEF_D, RED_REEF_E, RED_REEF_F, RED_REEF_G, RED_REEF_H, RED_REEF_I, RED_REEF_J, RED_REEF_K, RED_REEF_L};

    // Algae pick up locations
    public final static Pose2d BLUE_REEF_AB = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_CD = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_EF = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_GH = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_IJ = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_KL = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_AB = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_CD = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_EF = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_GH = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_IJ = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_KL = new Pose2d(1, 1, Rotation2d.fromDegrees(0));

    public final static Pose2d[] REEF_ALGAE_POSES = {BLUE_REEF_AB, BLUE_REEF_CD, BLUE_REEF_EF, BLUE_REEF_GH, BLUE_REEF_IJ, BLUE_REEF_KL,
        RED_REEF_AB, RED_REEF_CD, RED_REEF_EF, RED_REEF_GH, RED_REEF_IJ, RED_REEF_KL};

    // Algae scoring locations
    public final static Pose2d BLUE_PROCESSOR = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_PROCESSOR = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    // Would be cool to get net pathing so that it locks the x axis and rotation but lets us control the y axis
    public final static Pose2d BLUE_NET = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_NET = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    
}
