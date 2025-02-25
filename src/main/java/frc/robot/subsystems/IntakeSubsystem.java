package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFXS motorLeft, motorRight;

    private double velocity;
    private VelocityVoltage velocityRequest;
    private VoltageOut voltageRequest;

    private LinearFilter currentFilter;
    private double filteredCurrent;
    
    public IntakeSubsystem() {
        motorLeft = new TalonFXS(Ports.INTAKE_MOTOR_LEFT);
        motorRight = new TalonFXS(Ports.INTAKE_MOTOR_RIGHT);
        
        motorLeft.getConfigurator().apply(IntakeConstants.MOTOR_LEFT_CONFIG);

        motorRight.getConfigurator().apply(IntakeConstants.MOTOR_RIGHT_CONFIG);

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();
        motorLeft.getVelocity().setUpdateFrequency(20);
        motorRight.getVelocity().setUpdateFrequency(20);
        motorLeft.getStatorCurrent().setUpdateFrequency(20);
        motorRight.getStatorCurrent().setUpdateFrequency(20);
        motorLeft.getMotorVoltage().setUpdateFrequency(20);
        motorRight.getMotorVoltage().setUpdateFrequency(20);
        
        velocity = IntakeConstants.REST_VELOCITY;
        
        velocityRequest = new VelocityVoltage(velocity);

        motorLeft.setControl(velocityRequest.withVelocity(velocity));
        motorRight.setControl(velocityRequest.withVelocity(velocity));

        voltageRequest = new VoltageOut(0);

        currentFilter = LinearFilter.movingAverage(10);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = RobotContainer.getShuffleboardTab();

        tab.addDouble("Target Intake Velocity", () -> velocity);
        tab.addDouble("Actual Intake Velocity", () -> getVelocity());
        tab.addDouble("Filtered Intake Current", () -> filteredCurrent);
    }

    @Override
    public void periodic() {
        filteredCurrent = currentFilter.calculate(getCurrent());
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        motorLeft.setControl(velocityRequest.withVelocity(velocity));
        motorRight.setControl(velocityRequest.withVelocity(velocity));
    }

    public double getVelocity() {
        return motorLeft.getVelocity().getValueAsDouble();
    }

    public double getCurrent() {
        return motorLeft.getStatorCurrent().getValueAsDouble();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    public void setVoltage(double voltage) {
        motorLeft.setControl(voltageRequest.withOutput(voltage));
        motorRight.setControl(voltageRequest.withOutput(voltage));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.startEnd(() -> setVelocity(velocity), 
            () -> setVelocity(IntakeConstants.REST_VELOCITY));
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this)
            .finallyDo(() -> setVoltage(0));
    }

}
