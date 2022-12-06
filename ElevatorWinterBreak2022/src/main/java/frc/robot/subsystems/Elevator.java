package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {

    private static final double kCarriageMass = 4.5;

    private final ElevatorIO elevatorIO;
    private final ElevatorInputs elevatorInputs = new ElevatorInputs();

    public Elevator(ElevatorIO io) {
        this.elevatorIO = io;
    }

}
