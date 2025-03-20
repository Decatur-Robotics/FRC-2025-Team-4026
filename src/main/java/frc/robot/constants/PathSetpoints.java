package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathSetpoints {

    // 0.6858 + 0.375
    // id 7: 13.890, 4.026
    // robot: 14.96, 4.026

    // Robot chassis edge to center 0.752475 meters

    // Robot center should be 0.456475 meters from reef face for scoring

    // 0.164338 meters from apriltag to reef branch on parallel plane

    // Field center
    public static final Translation2d FIELD_CENTER = new Translation2d(8.774176, 4.026281);

    // Apriltag poses
    public final static Pose2d APRILTAG_1 = new Pose2d(16.7, 0.66, Rotation2d.fromDegrees(126));
    public final static Pose2d APRILTAG_2 = new Pose2d(16.7, 7.4, Rotation2d.fromDegrees(234));
    public final static Pose2d APRILTAG_3 = new Pose2d(11.56, 8.06, Rotation2d.fromDegrees(270));
    public final static Pose2d APRILTAG_4 = new Pose2d(9.28, 6.14, Rotation2d.fromDegrees(0));
    public final static Pose2d APRILTAG_5 = new Pose2d(9.28, 1.91, Rotation2d.fromDegrees(0));
    public final static Pose2d APRILTAG_6 = new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(300));
    public final static Pose2d APRILTAG_7 = new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(0));
    public final static Pose2d APRILTAG_8 = new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(60));
    public final static Pose2d APRILTAG_9 = new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(120));
    public final static Pose2d APRILTAG_10 = new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(180));
    public final static Pose2d APRILTAG_11 = new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(240));
    public final static Pose2d APRILTAG_12 = new Pose2d(0.85, 0.66, Rotation2d.fromDegrees(54));
    public final static Pose2d APRILTAG_13 = new Pose2d(0.85, 7.4, Rotation2d.fromDegrees(306));
    public final static Pose2d APRILTAG_14 = new Pose2d(8.27, 6.14, Rotation2d.fromDegrees(180));
    public final static Pose2d APRILTAG_15 = new Pose2d(8.27, 1.91, Rotation2d.fromDegrees(180));
    public final static Pose2d APRILTAG_16 = new Pose2d(5.99, 0, Rotation2d.fromDegrees(90));
    public final static Pose2d APRILTAG_17 = new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(240));
    public final static Pose2d APRILTAG_18 = new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(180));
    public final static Pose2d APRILTAG_19 = new Pose2d(4.07, 4.75, Rotation2d.fromDegrees(120));
    public final static Pose2d APRILTAG_20 = new Pose2d(4.9, 4.75, Rotation2d.fromDegrees(60));
    public final static Pose2d APRILTAG_21 = new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(0));
    public final static Pose2d APRILTAG_22 = new Pose2d(4.9, 3.31, Rotation2d.fromDegrees(300));

    // Coral scoring locations
    public final static Pose2d BLUE_REEF_A = new Pose2d(3.2011, 4.1902, Rotation2d.fromDegrees(180));
    public final static Pose2d BLUE_REEF_B = new Pose2d(3.2011, 3.8616, Rotation2d.fromDegrees(180));
    public final static Pose2d BLUE_REEF_C = new Pose2d(3.7034, 2.9932, Rotation2d.fromDegrees(240));
    public final static Pose2d BLUE_REEF_D = new Pose2d(3.9880, 2.8288, Rotation2d.fromDegrees(240));
    public final static Pose2d BLUE_REEF_E = new Pose2d(4.9907, 2.8288, Rotation2d.fromDegrees(300));
    public final static Pose2d BLUE_REEF_F = new Pose2d(5.2753, 2.9932, Rotation2d.fromDegrees(300));
    public final static Pose2d BLUE_REEF_G = new Pose2d(5.7775, 3.8616, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_H = new Pose2d(5.7775, 4.1902, Rotation2d.fromDegrees(0));
    public final static Pose2d BLUE_REEF_I = new Pose2d(5.2753, 5.0586, Rotation2d.fromDegrees(60));
    public final static Pose2d BLUE_REEF_J = new Pose2d(4.9907, 5.2230, Rotation2d.fromDegrees(60));
    public final static Pose2d BLUE_REEF_K = new Pose2d(3.9880, 5.2230, Rotation2d.fromDegrees(120));
    public final static Pose2d BLUE_REEF_L = new Pose2d(3.7034, 5.0586, Rotation2d.fromDegrees(120));

    public final static Pose2d RED_REEF_A = new Pose2d(14.46, 3.86, Rotation2d.fromDegrees(0)); // x: 14.46
    public final static Pose2d RED_REEF_B = new Pose2d(14.46, 4.19, Rotation2d.fromDegrees(0));
    public final static Pose2d RED_REEF_C = new Pose2d(13.8450, 5.0586, Rotation2d.fromDegrees(60));
    public final static Pose2d RED_REEF_D = new Pose2d(13.5604, 5.2230, Rotation2d.fromDegrees(60));
    public final static Pose2d RED_REEF_E = new Pose2d(12.5575, 5.2230, Rotation2d.fromDegrees(120));
    public final static Pose2d RED_REEF_F = new Pose2d(12.2728, 5.0586, Rotation2d.fromDegrees(120));
    public final static Pose2d RED_REEF_G = new Pose2d(11.7709, 4.1902, Rotation2d.fromDegrees(180));
    public final static Pose2d RED_REEF_H = new Pose2d(11.7709, 3.8616, Rotation2d.fromDegrees(180));
    public final static Pose2d RED_REEF_I = new Pose2d(12.2728, 2.9932, Rotation2d.fromDegrees(240));
    public final static Pose2d RED_REEF_J = new Pose2d(12.5575, 2.8288, Rotation2d.fromDegrees(240));
    public final static Pose2d RED_REEF_K = new Pose2d(13.5604, 2.8288, Rotation2d.fromDegrees(300));
    public final static Pose2d RED_REEF_L = new Pose2d(13.8450, 2.9932, Rotation2d.fromDegrees(300));

    public final static Pose2d[] CORAL_SCORING_POSES = {BLUE_REEF_A, BLUE_REEF_B, BLUE_REEF_C, BLUE_REEF_D, BLUE_REEF_E, BLUE_REEF_F, BLUE_REEF_G, BLUE_REEF_H, BLUE_REEF_I, BLUE_REEF_J, BLUE_REEF_K, BLUE_REEF_L,
        RED_REEF_A, RED_REEF_B, RED_REEF_C, RED_REEF_D, RED_REEF_E, RED_REEF_F, RED_REEF_G, RED_REEF_H, RED_REEF_I, RED_REEF_J, RED_REEF_K, RED_REEF_L};

    // Algae pick up locations
    public final static Pose2d BLUE_REEF_AB = BLUE_REEF_A.interpolate(BLUE_REEF_B, 0.5);
    public final static Pose2d BLUE_REEF_CD = BLUE_REEF_C.interpolate(BLUE_REEF_D, 0.5);
    public final static Pose2d BLUE_REEF_EF = BLUE_REEF_E.interpolate(BLUE_REEF_F, 0.5);
    public final static Pose2d BLUE_REEF_GH = BLUE_REEF_G.interpolate(BLUE_REEF_H, 0.5);
    public final static Pose2d BLUE_REEF_IJ = BLUE_REEF_I.interpolate(BLUE_REEF_J, 0.5);
    public final static Pose2d BLUE_REEF_KL = BLUE_REEF_K.interpolate(BLUE_REEF_L, 0.5);
    public final static Pose2d RED_REEF_AB = RED_REEF_A.interpolate(RED_REEF_B, 0.5);
    public final static Pose2d RED_REEF_CD = RED_REEF_C.interpolate(RED_REEF_D, 0.5);
    public final static Pose2d RED_REEF_EF = RED_REEF_E.interpolate(RED_REEF_F, 0.5);
    public final static Pose2d RED_REEF_GH = RED_REEF_G.interpolate(RED_REEF_H, 0.5);
    public final static Pose2d RED_REEF_IJ = RED_REEF_I.interpolate(RED_REEF_J, 0.5);
    public final static Pose2d RED_REEF_KL = RED_REEF_K.interpolate(RED_REEF_L, 0.5);

    public final static Pose2d[] REEF_ALGAE_POSES = {BLUE_REEF_AB, BLUE_REEF_CD, BLUE_REEF_EF, BLUE_REEF_GH, BLUE_REEF_IJ, BLUE_REEF_KL,
        RED_REEF_AB, RED_REEF_CD, RED_REEF_EF, RED_REEF_GH, RED_REEF_IJ, RED_REEF_KL};

    // Algae scoring locations
    public final static Pose2d BLUE_PROCESSOR = new Pose2d(5.988, 0.634, Rotation2d.fromDegrees(270)); // y = 0.634365
    public final static Pose2d RED_PROCESSOR = new Pose2d(11.560, 7.417, Rotation2d.fromDegrees(90)); // y = 7.417435
    // Would be cool to get net pathing so that it locks the x axis and rotation but lets us control the y axis
    public final static Pose2d BLUE_NET = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    public final static Pose2d RED_NET = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    // Human player locations
    public final static Pose2d BLUE_LEFT_HUMAN_PLAYER = new Pose2d(1.12, 7.03, Rotation2d.fromDegrees(306));
    public final static Pose2d BLUE_RIGHT_HUMAN_PLAYER = new Pose2d(1.12, 1.02, Rotation2d.fromDegrees(54));
    public final static Pose2d RED_LEFT_HUMAN_PLAYER = new Pose2d(16.43, 1.02, Rotation2d.fromDegrees(126));
    public final static Pose2d RED_RIGHT_HUMAN_PLAYER = new Pose2d(16.43, 7.03, Rotation2d.fromDegrees(234));
    
}
