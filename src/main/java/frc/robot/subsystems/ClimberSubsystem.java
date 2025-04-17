package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class ClimberSubsystem extends SubsystemBase{

    private TalonFX climberMotor;
    
    private double position;
    private PositionVoltage controlRequest;

    private VoltageOut voltageRequest;
    
    public ClimberSubsystem() {
        climberMotor = new TalonFX(Ports.CLIMBER_MOTOR, "CANivore 0");

        climberMotor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIG);

        // CAN optimization
        climberMotor.optimizeBusUtilization();
        climberMotor.getRotorPosition().setUpdateFrequency(20);
        
        position = ClimberConstants.INITIAL_POSITION;

        controlRequest = new PositionVoltage(position);
        climberMotor.setControl(controlRequest);

        voltageRequest = new VoltageOut(0);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_CLIMBER_TAB);

        tab.addDouble("Target Climber Position", () -> position);
        tab.addDouble("Actual Climber Position", () -> getPosition());
        tab.addDouble("Actual Climber Voltage", () -> getVoltage());
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

    public void setVoltage(double voltage) {
        climberMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public double getVoltage() {
        return climberMotor.getMotorVoltage().getValueAsDouble();
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

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this)
            .finallyDo(() -> climberMotor.setControl(new NeutralOut()));
    }

}


