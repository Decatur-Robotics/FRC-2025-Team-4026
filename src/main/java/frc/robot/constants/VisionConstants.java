package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {

    // Apriltag ID 18 is 12.13 inches off the ground
    // 14.625 inches from center of robot to edge
    
    public static final String CAMERA_FRONT_LEFT_NAME = "FrontLeftCamera"; // 2
    public static final String CAMERA_FRONT_RIGHT_NAME = "FrontRightCamera"; // 3
    public static final String CAMERA_BACK_NAME = "BackCamera"; // 1

    public static final Transform3d ROBOT_TO_CAMERA_FRONT_LEFT = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));
    public static final Transform3d ROBOT_TO_CAMERA_FRONT_RIGHT = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));
    public static final Transform3d ROBOT_TO_CAMERA_BACK = new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0));

    public static final Matrix<N3, N1> SINGLE_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0, 0, 0);
    public static final Matrix<N3, N1> MULTI_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0, 0, 0);

}
