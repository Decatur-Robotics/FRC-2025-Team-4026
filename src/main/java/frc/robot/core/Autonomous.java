package frc.robot.core;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.PathSetpoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain.PathLocation;

public class Autonomous {

    private enum AutoSide {
        Left("Left Side"), Center("Center"), Right("Right Side");

        private final String autoName;

        private AutoSide(String autoName) {
            this.autoName = autoName;
        }
    }

    private enum AutoType {
        ThreeCoralHumanPlayer("Three Coral Human Player"), OneCoral("One Coral");

        private final String autoName;

        private AutoType(String autoName) {
            this.autoName = autoName;
        }
    }

    private SendableChooser<AutoSide> autoSideChooser;
    private SendableChooser<AutoType> autoTypeChooser;

    private Command autoCommand;

    private RobotContainer robotContainer;

    private final CommandSwerveDrivetrain swerve;
    private final SuperstructureSubsystem superstructure;

    public Autonomous(final RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        swerve = robotContainer.getSwerve();
        superstructure = robotContainer.getSuperstructure();

        registerNamedCommands();
        
        autoSideChooser = new SendableChooser<>();
        autoSideChooser.setDefaultOption(AutoSide.Center.autoName, AutoSide.Center);
        autoSideChooser.addOption(AutoSide.Left.autoName, AutoSide.Left);
        autoSideChooser.addOption(AutoSide.Center.autoName, AutoSide.Center);
        autoSideChooser.addOption(AutoSide.Right.autoName, AutoSide.Right);

        autoTypeChooser = new SendableChooser<>();
        autoTypeChooser.setDefaultOption(AutoType.ThreeCoralHumanPlayer.autoName, AutoType.ThreeCoralHumanPlayer);
        autoTypeChooser.addOption(AutoType.ThreeCoralHumanPlayer.autoName, AutoType.ThreeCoralHumanPlayer);
        autoTypeChooser.addOption(AutoType.OneCoral.autoName, AutoType.OneCoral);

        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        autoTab.add(autoSideChooser);
        autoTab.add(autoTypeChooser);

        autoSideChooser.onChange((autoSide) -> updateAutoCommand());
        autoTypeChooser.onChange((autoSide) -> updateAutoCommand());

        updateAutoCommand();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Score C", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_C, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_C, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Score D", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_D, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_D, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Score E", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_E, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_E, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Score J", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_J, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_J, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Score K", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_K, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_K, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Score L", 
            Commands.deadline(autoL4Command(), 
                Commands.either(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_L, PathLocation.Reef), 
                    swerve.driveToPoseAuto(PathSetpoints.RED_REEF_L, PathLocation.Reef), 
                    () -> DriverStation.getAlliance().get().equals(Alliance.Blue))));

        NamedCommands.registerCommand("Intake", superstructure.intakeCoralHumanPlayerCommand());
    }

    public Command getAutoCommand() {
        return autoCommand;
        // return new PathPlannerAuto("Test J to Left Source");
    }

    public void updateAutoCommand() {
        if (autoTypeChooser.getSelected().equals(AutoType.ThreeCoralHumanPlayer)) {
            if (autoSideChooser.getSelected().equals(AutoSide.Left)) {
                // autoCommand = new PathPlannerAuto("Left Side 3 Coral");
                autoCommand = leftThreeCoralHumanPlayerAuto();
            }
            else if (autoSideChooser.getSelected().equals(AutoSide.Right)) {
                // autoCommand = new PathPlannerAuto("Right Side 3 Coral");
                autoCommand = rightThreeCoralHumanPlayerAuto();
            }
            else {
                autoCommand = centerOneCoralAuto();
            }
        }
        else {
            if (autoSideChooser.getSelected().equals(AutoSide.Left)) {
                autoCommand = leftOneCoralAuto();
            }
            else if (autoSideChooser.getSelected().equals(AutoSide.Right)) {
                autoCommand = rightOneCoralAuto();
            }
            else {
                autoCommand = centerOneCoralAuto();
            }
        }

        FollowPathCommand.warmupCommand().schedule();
    }

    private Command autoL4Command() {
        return superstructure.scoreCoralL4Command(() -> swerve.isNearAligned(), () -> swerve.isAligned(), 
            () -> (Robot.isSimulation() && swerve.isNearAligned()), 
            () -> (Robot.isSimulation() && swerve.isAligned()));
    }

    //
    // Auto commands
    //

    private Command leftOneCoralAuto() {
        return Commands.either(
            oneCoralAuto(AutoConstants.BLUE_LEFT_INITIAL_POSE, PathSetpoints.BLUE_REEF_J),
            oneCoralAuto(AutoConstants.RED_LEFT_INITIAL_POSE, PathSetpoints.RED_REEF_J),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue));
    }

    private Command centerOneCoralAuto() {
        return Commands.either(
            oneCoralAuto(AutoConstants.BLUE_CENTER_INITIAL_POSE, PathSetpoints.BLUE_REEF_G),
            oneCoralAuto(AutoConstants.RED_CENTER_INITIAL_POSE, PathSetpoints.RED_REEF_G),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue));
    }

    private Command rightOneCoralAuto() {
        return Commands.either(
            oneCoralAuto(AutoConstants.BLUE_RIGHT_INITIAL_POSE, PathSetpoints.BLUE_REEF_E),
            oneCoralAuto(AutoConstants.RED_RIGHT_INITIAL_POSE, PathSetpoints.RED_REEF_E),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue));
    }

    private Command leftThreeCoralHumanPlayerAuto() {
        return Commands.either(
            threeCoralHumanPlayerAuto(AutoConstants.BLUE_LEFT_INITIAL_POSE, PathSetpoints.BLUE_LEFT_HUMAN_PLAYER_LEFT, 
                PathSetpoints.BLUE_REEF_J, PathSetpoints.BLUE_REEF_L, PathSetpoints.BLUE_REEF_K),
            threeCoralHumanPlayerAuto(AutoConstants.RED_LEFT_INITIAL_POSE, PathSetpoints.RED_LEFT_HUMAN_PLAYER_LEFT, 
                PathSetpoints.RED_REEF_J, PathSetpoints.RED_REEF_L, PathSetpoints.RED_REEF_K),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue));
    }

    private Command centerThreeCoralHumanPlayerAuto() {
        // No three coral for center

        return centerOneCoralAuto();
    }

    private Command rightThreeCoralHumanPlayerAuto() {
        return Commands.either(
            threeCoralHumanPlayerAuto(AutoConstants.BLUE_RIGHT_INITIAL_POSE, PathSetpoints.BLUE_RIGHT_HUMAN_PLAYER_RIGHT, 
                PathSetpoints.BLUE_REEF_E, PathSetpoints.BLUE_REEF_C, PathSetpoints.BLUE_REEF_D),
            threeCoralHumanPlayerAuto(AutoConstants.RED_RIGHT_INITIAL_POSE, PathSetpoints.RED_RIGHT_HUMAN_PLAYER_RIGHT, 
                PathSetpoints.RED_REEF_E, PathSetpoints.RED_REEF_C, PathSetpoints.RED_REEF_D),
            () -> DriverStation.getAlliance().get().equals(Alliance.Blue));
    }

    private Command oneCoralAuto(Pose2d initialPose, Pose2d branchPose) {
        return Commands.sequence(
            Commands.runOnce(() -> swerve.resetPose(initialPose), swerve),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(branchPose, PathLocation.Reef))
        );
    }

    private Command threeCoralHumanPlayerAuto(Pose2d initialPose, Pose2d humanPlayerPose, 
            Pose2d firstBranchPose, Pose2d secondBranchPose, Pose2d thirdBranchPose) {
        return Commands.sequence(
            Commands.runOnce(() -> swerve.resetPose(initialPose), swerve),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(firstBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToHumanPlayerFromReefBacksideAuto(humanPlayerPose)),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(secondBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToPoseAuto(humanPlayerPose, PathLocation.HumanPlayer)),
            Commands.deadline(
                autoL4Command(),
                swerve.driveToPoseAuto(thirdBranchPose, PathLocation.Reef)),
            Commands.deadline(
                superstructure.intakeCoralHumanPlayerCommand(), 
                swerve.driveToPoseAuto(humanPlayerPose, PathLocation.HumanPlayer))
        );
    }
        
}
