package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathSetpoints {

    // Robot center should be 0.57 meters from reef face for scoring

    // 0.164338 meters from apriltag to reef branch on parallel plane    

    public static final Transform2d LEFT_BRANCH_OFFSET = new Transform2d(new Translation2d(0.57, -0.164), Rotation2d.kZero);
    public static final Transform2d RIGHT_BRANCH_OFFSET = new Transform2d(new Translation2d(0.57, 0.164), Rotation2d.kZero);

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
    public final static Pose2d BLUE_REEF_A = APRILTAG_18.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_B = APRILTAG_18.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_C = APRILTAG_17.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_D = APRILTAG_17.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_E = APRILTAG_22.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_F = APRILTAG_22.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_G = APRILTAG_21.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_H = APRILTAG_21.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_I = APRILTAG_20.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_J = APRILTAG_20.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_K = APRILTAG_19.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d BLUE_REEF_L = APRILTAG_19.transformBy(RIGHT_BRANCH_OFFSET);

    public final static Pose2d RED_REEF_A = APRILTAG_7.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_B = APRILTAG_7.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_C = APRILTAG_8.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_D = APRILTAG_8.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_E = APRILTAG_9.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_F = APRILTAG_9.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_G = APRILTAG_10.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_H = APRILTAG_10.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_I = APRILTAG_11.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_J = APRILTAG_11.transformBy(RIGHT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_K = APRILTAG_6.transformBy(LEFT_BRANCH_OFFSET);
    public final static Pose2d RED_REEF_L = APRILTAG_6.transformBy(RIGHT_BRANCH_OFFSET);

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

    // Human player locations relative to driver station
    public final static double HUMAN_PLAYER_ROBOT_OFFSET = 0.45;
    public final static double HUMAN_PLAYER_SIDE_OFFSET = 0.45;
    // Blue left HP
    public final static Pose2d BLUE_LEFT_HUMAN_PLAYER_LEFT = APRILTAG_13.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    public final static Pose2d BLUE_LEFT_HUMAN_PLAYER_CENTER = APRILTAG_13.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, 0, Rotation2d.kZero));
    public final static Pose2d BLUE_LEFT_HUMAN_PLAYER_RIGHT = APRILTAG_13.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, -HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    // Blue right HP
    public final static Pose2d BLUE_RIGHT_HUMAN_PLAYER_LEFT = APRILTAG_12.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    public final static Pose2d BLUE_RIGHT_HUMAN_PLAYER_CENTER = APRILTAG_12.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, 0, Rotation2d.kZero));
    public final static Pose2d BLUE_RIGHT_HUMAN_PLAYER_RIGHT = APRILTAG_12.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, -HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    // Red left HP
    public final static Pose2d RED_LEFT_HUMAN_PLAYER_LEFT = APRILTAG_1.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    public final static Pose2d RED_LEFT_HUMAN_PLAYER_CENTER = APRILTAG_1.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, 0, Rotation2d.kZero));
    public final static Pose2d RED_LEFT_HUMAN_PLAYER_RIGHT = APRILTAG_1.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, -HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    // Red right HP
    public final static Pose2d RED_RIGHT_HUMAN_PLAYER_LEFT = APRILTAG_2.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));
    public final static Pose2d RED_RIGHT_HUMAN_PLAYER_CENTER = APRILTAG_2.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, 0, Rotation2d.kZero));
    public final static Pose2d RED_RIGHT_HUMAN_PLAYER_RIGHT = APRILTAG_2.transformBy(new Transform2d(HUMAN_PLAYER_ROBOT_OFFSET, -HUMAN_PLAYER_SIDE_OFFSET, Rotation2d.kZero));

    public static final Pose2d[] HUMAN_PLAYER_POSES = {BLUE_LEFT_HUMAN_PLAYER_LEFT, BLUE_LEFT_HUMAN_PLAYER_CENTER, BLUE_LEFT_HUMAN_PLAYER_RIGHT,
        BLUE_RIGHT_HUMAN_PLAYER_LEFT, BLUE_RIGHT_HUMAN_PLAYER_CENTER, BLUE_RIGHT_HUMAN_PLAYER_RIGHT,
        RED_LEFT_HUMAN_PLAYER_LEFT, RED_LEFT_HUMAN_PLAYER_CENTER, RED_LEFT_HUMAN_PLAYER_RIGHT,
        RED_RIGHT_HUMAN_PLAYER_LEFT, RED_RIGHT_HUMAN_PLAYER_CENTER, RED_RIGHT_HUMAN_PLAYER_RIGHT};

    // Climb poses
    public static final Pose2d BLUE_LEFT_CAGE = new Pose2d(8.79, 7.26, Rotation2d.kZero);
    public static final Pose2d BLUE_CENTER_CAGE = new Pose2d(8.79, 6.16, Rotation2d.kZero);
    public static final Pose2d BLUE_RIGHT_CAGE = new Pose2d(8.79, 5.06, Rotation2d.kZero);
    public static final Pose2d RED_LEFT_CAGE = new Pose2d(8.79, 0.79, Rotation2d.k180deg);
    public static final Pose2d RED_CENTER_CAGE = new Pose2d(8.79, 1.89, Rotation2d.k180deg);
    public static final Pose2d RED_RIGHT_CAGE = new Pose2d(8.79, 2.99, Rotation2d.k180deg);

    public static final Transform2d CLIMB_STAGING_OFFSET = new Transform2d(new Translation2d(-0.5, 0), Rotation2d.kZero);
    public static final Transform2d CLIMB_ALIGNING_OFFSET = new Transform2d(new Translation2d(0.2, 0), Rotation2d.kZero);

    public final static Pose2d BLUE_LEFT_CAGE_STAGE = BLUE_LEFT_CAGE.transformBy(CLIMB_STAGING_OFFSET);
    public final static Pose2d BLUE_CENTER_CAGE_STAGE = BLUE_CENTER_CAGE.transformBy(CLIMB_STAGING_OFFSET);
    public final static Pose2d BLUE_RIGHT_CAGE_STAGE = BLUE_RIGHT_CAGE.transformBy(CLIMB_STAGING_OFFSET);
    public final static Pose2d RED_LEFT_CAGE_STAGE = RED_LEFT_CAGE.transformBy(CLIMB_STAGING_OFFSET);
    public final static Pose2d RED_CENTER_CAGE_STAGE = RED_CENTER_CAGE.transformBy(CLIMB_STAGING_OFFSET);
    public final static Pose2d RED_RIGHT_CAGE_STAGE = RED_RIGHT_CAGE.transformBy(CLIMB_STAGING_OFFSET);

    public static final Pose2d[] CAGE_STAGE_POSES = {BLUE_LEFT_CAGE_STAGE, BLUE_CENTER_CAGE_STAGE, BLUE_RIGHT_CAGE_STAGE,
        RED_LEFT_CAGE_STAGE, RED_CENTER_CAGE_STAGE, RED_RIGHT_CAGE_STAGE};

    public final static Pose2d BLUE_LEFT_CAGE_ALIGN = BLUE_LEFT_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);
    public final static Pose2d BLUE_CENTER_CAGE_ALIGN = BLUE_CENTER_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);
    public final static Pose2d BLUE_RIGHT_CAGE_ALIGN = BLUE_RIGHT_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);
    public final static Pose2d RED_LEFT_CAGE_ALIGN = RED_LEFT_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);
    public final static Pose2d RED_CENTER_CAGE_ALIGN = RED_CENTER_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);
    public final static Pose2d RED_RIGHT_CAGE_ALIGN = RED_RIGHT_CAGE.transformBy(CLIMB_ALIGNING_OFFSET);

    public static final Pose2d[] CAGE_ALIGN_POSES = {BLUE_LEFT_CAGE_ALIGN, BLUE_CENTER_CAGE_ALIGN, BLUE_RIGHT_CAGE_ALIGN,
        RED_LEFT_CAGE_ALIGN, RED_CENTER_CAGE_ALIGN, RED_RIGHT_CAGE_ALIGN};
    
}
