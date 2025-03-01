package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathSetpoints {

    // Field center
    public static final Translation2d FIELD_CENTER = new Translation2d(8.774176, 4.026281);

    public static final Rotation2d FLIP_ROTATION = new Rotation2d(Degrees.of(180));

    // Coral scoring locations
    public final static Pose2d REEF_A = new Pose2d(3.2, 4.2, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_B = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_C = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_D = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_E = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_F = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_G = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_H = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_I = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_J = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_K = new Pose2d(99, 99, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_L = new Pose2d(99, 99, Rotation2d.fromDegrees(0));

    public final static Pose2d[] CORAL_SCORING_POSES = {REEF_A, REEF_B, REEF_C, REEF_D, REEF_E, REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L};

    // Algae pick up locations
    public final static Pose2d REEF_AB = new Pose2d(1, 5, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_CD = new Pose2d(2, 10, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_EF = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_GH = new Pose2d(10, 0, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_IJ = new Pose2d(10, 10, Rotation2d.fromDegrees(0));
    public final static Pose2d REEF_KL = new Pose2d(15, 5, Rotation2d.fromDegrees(0));

    public final static Pose2d[] REEF_ALGAE_POSES = {REEF_AB, REEF_CD, REEF_EF, REEF_GH, REEF_IJ, REEF_KL};

    // Algae scoring locations
    public final static Pose2d PROCESSOR = new Pose2d(20, 20, Rotation2d.fromDegrees(0));
    // Would be cool to get net pathing so that it locks the x axis and rotation but lets us control the y axis
    public final static Pose2d NET = new Pose2d(20, 20, Rotation2d.fromDegrees(0));
    
}
