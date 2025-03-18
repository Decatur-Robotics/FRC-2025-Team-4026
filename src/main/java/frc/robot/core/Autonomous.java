package frc.robot.core;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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

    public Autonomous(final RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        
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
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public void updateAutoCommand() {
        autoCommand = Commands.print("Running auto...");

        if (autoTypeChooser.getSelected().equals(AutoType.ThreeCoralHumanPlayer)) {
            if (autoSideChooser.getSelected().equals(AutoSide.Left)) {
            
            }
            else if (autoSideChooser.getSelected().equals(AutoSide.Right)) {
    
            }
            else {
    
            }
        }
        else {
            if (autoSideChooser.getSelected().equals(AutoSide.Left)) {
            
            }
            else if (autoSideChooser.getSelected().equals(AutoSide.Right)) {
    
            }
            else {
    
            }
        }
    }

    // private Command leftOnePieceAuto() {
        
    // }

    // private Command centerOnePieceAuto() {

    // }
        
}
