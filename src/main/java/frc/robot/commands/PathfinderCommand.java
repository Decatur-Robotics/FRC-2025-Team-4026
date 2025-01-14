package frc.robot.commands;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import frc.robot.constants.PathSetpoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfinderCommand {    

       private Pose2d robotPose;
        private Pose2d targetPose;
        private PathConstraints constraints;

        private PathPlannerPath path;

       

 public PathfinderCommand() {
    targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    robotPose = new Pose2d(0 ,0 , Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
                  3.0, 4.0,
                 Units.degreesToRadians(540), Units.degreesToRadians(720));
 }
   
    
    public Pose2d findBestTarget(double poseNum, PathSetpoints path){
        //TODO: have this end when path is generated

        Pose2d[] poses = {path.REEF_A, path.REEF_B, path.REEF_C, path.REEF_D, path.REEF_E, path.REEF_F, path.REEF_G, path.REEF_H, path.REEF_I, path.REEF_J, path.REEF_K, path.REEF_L};
        poseNum = poses.length;

        int left = 0, right = (int)poseNum - 1;
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
