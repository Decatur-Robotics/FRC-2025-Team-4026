package frc.robot.core;

import java.io.IOException;

import javax.xml.stream.events.Comment;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class Pathfinder{

    
        private Pose2d targetPose;
        private PathConstraints constraints;

        private PathPlannerPath path;

    public Pathfinder() throws FileVersionException, IOException, ParseException{
        targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        path = PathPlannerPath.fromPathFile("Align A");
        // Create the constraints to use while pathfinding
        constraints = new PathConstraints(
                  3.0, 4.0,
                 Units.degreesToRadians(540), Units.degreesToRadians(720));

    }

    public void findToPath(){
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public void findToPose(){
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0
        );
    }


}
