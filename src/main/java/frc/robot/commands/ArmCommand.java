package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command
{
	public ArmSubsystem arm;
	public double targetAngle;

	public ArmCommand(ArmSubsystem arm, double targetAngle)
	{		
		this.targetAngle = targetAngle;

		this.arm = arm;
		addRequirements(arm);
	
	}

    public void setToIntakeAngle(){
        targetAngle = ArmConstants.ARM_INTAKE_POSITION;
    }

    public void setToL1Position(){
        targetAngle = ArmConstants.ARM_L1_POSITION;
    }

    public void setToL2Position(){
        targetAngle = ArmConstants.ARM_L2_POSITION;
    }

    public void setToL3Position(){
        targetAngle = ArmConstants.ARM_L3_POSITION;
    }

    public void setToL4Position(){
        targetAngle = ArmConstants.ARM_L4_POSITION;
    }

    public void setToSourcePosition(){
        targetAngle = ArmConstants.ARM_SOURCE_POSITION;
    }

	@Override
	public void initialize(){
		arm.setTargetRotation(targetAngle);
	}
}