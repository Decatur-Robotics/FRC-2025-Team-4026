package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX climberMotor;
    
    private double position;
    private MotionMagicDutyCycle controlRequest;
    
    public ClimberSubsystem() {
        climberMotor = new TalonFX(Ports.CLIMBER_MOTOR, "CANivore 0");

        climberMotor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIG);

        // CAN optimization
        climberMotor.optimizeBusUtilization();
        climberMotor.getRotorPosition().setUpdateFrequency(20);
        
        position = ClimberConstants.INITIAL_POSITION;

        controlRequest = new MotionMagicDutyCycle(position);
        climberMotor.setControl(controlRequest);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = RobotContainer.getShuffleboardTab();

        tab.addDouble("Target Climber Position", () -> position);
        tab.addDouble("Target Climber Position", () -> getPosition());
    }

    
    @Override
    public void periodic() {
        if (climberMotor.hasResetOccurred()) {
			climberMotor.optimizeBusUtilization();
			climberMotor.getRotorPosition().setUpdateFrequency(20);
		}
    }
    
    /**
     * Sets the position of the climber motor based on a target position
     * @param position
     */
    public void setPosition(double position) {
        this.position = position;
        climberMotor.setControl(controlRequest.withPosition(this.position));
    }

    /**
     * @return Position of the motor in mechanism rotations
     */
    public double getPosition() {
        return climberMotor.getRotorPosition().getValueAsDouble();
    }

    /**
     * Moves the climber to an upright position, waits for an interrupt, then moves back down
     */
    public Command climbCommand() {
        return startEnd(
            () -> this.setPosition(ClimberConstants.CLIMBED_POSITION),
            () -> this.setPosition(ClimberConstants.INITIAL_POSITION));
    }

}


