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
    public final static Pose2d BLUE_LEFT_HUMAN_PLAYER = new Pose2d(0.851, 7.396, Rotation2d.fromDegrees(306));
    public final static Pose2d BLUE_RIGHT_HUMAN_PLAYER = new Pose2d(0.851, 0.655, Rotation2d.fromDegrees(54));
    public final static Pose2d RED_LEFT_HUMAN_PLAYER = new Pose2d(16.697, 0.655, Rotation2d.fromDegrees(126));
    public final static Pose2d RED_RIGHT_HUMAN_PLAYER = new Pose2d(16.697, 7.396, Rotation2d.fromDegrees(234));
    
}
