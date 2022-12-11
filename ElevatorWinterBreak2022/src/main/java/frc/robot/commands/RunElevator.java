package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class RunElevator extends CommandBase {

    Elevator m_elevator;
    DoubleSupplier m_speedSupplier;

    public RunElevator(Elevator elevator, DoubleSupplier speedSupplier) {
        this.m_elevator = elevator;
        this.m_speedSupplier = speedSupplier;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_elevator.setSpeed(m_speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
