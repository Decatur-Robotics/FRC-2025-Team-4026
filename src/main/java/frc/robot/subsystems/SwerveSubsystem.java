package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
    private Pose2d getPose;
    public SwerveSubsystem(){
        //Do other swerve stuff here

            RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
		// Type::method gets a reference to the method. Type.method only allows us to
		// run the method
		AutoBuilder.configure(this::getPose, this::resetPose, this::getCurrentSpeeds,
				this::drive, (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), 
                        new PIDConstants(5.0, 0.0, 0.0), config, () -> {var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                              return alliance.get() == DriverStation.Alliance.Red;
                            }
                            return false;
                          },
                          this));
                        
	

    }
}
