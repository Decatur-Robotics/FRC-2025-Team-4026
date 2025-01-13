package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfinderCommand {    

       private Pose2d robotPose;
        private Pose2d targetPose;
        private PathConstraints constraints;

        private PathPlannerPath path;

        private Pose2d A;
        private Pose2d B;
        private Pose2d C;
        private Pose2d D;
        private Pose2d E;
        private Pose2d F;
        private Pose2d G;
        private Pose2d H;
        private Pose2d I;
        private Pose2d J;
        private Pose2d K;
        private Pose2d L;

 public PathfinderCommand() {
    targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    robotPose = new Pose2d(0 ,0 , Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
                  3.0, 4.0,
                 Units.degreesToRadians(540), Units.degreesToRadians(720));
 }
   
    
     public Pose2d findBestTarget(double poseNum){
        A = new Pose2d(1, 5, Rotation2d.fromDegrees(0));
        B = new Pose2d(0, 5, Rotation2d.fromDegrees(0));
        C = new Pose2d(2, 10, Rotation2d.fromDegrees(0));
        D = new Pose2d(5, 2, Rotation2d.fromDegrees(0));
        E = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
        F = new Pose2d(5, 10, Rotation2d.fromDegrees(0));
        G = new Pose2d(10, 0, Rotation2d.fromDegrees(0));
        H = new Pose2d(10, 5, Rotation2d.fromDegrees(0));
        I = new Pose2d(10, 10, Rotation2d.fromDegrees(0));
        J = new Pose2d(15, 0, Rotation2d.fromDegrees(0));
        K = new Pose2d(15, 5, Rotation2d.fromDegrees(0));
        L = new Pose2d(15, 10, Rotation2d.fromDegrees(0));

        Pose2d[] poses = {A, B, C, D, E, F, G, H, I, J, K, L};
        poseNum = poses.length;

        int left = 0, right = (int)poseNum - 1;
        //TODO: make it so this only runs when a button if held down
        while(left < right){
            if(poses[left].getTranslation().getDistance(robotPose.getTranslation())
                <= poses[right].getTranslation().getDistance(robotPose.getTranslation())){
                    right--;
            }
            else{
                left++;
            }
        }
        targetPose = poses[left];
        return poses[left];
       
    }
 }   
