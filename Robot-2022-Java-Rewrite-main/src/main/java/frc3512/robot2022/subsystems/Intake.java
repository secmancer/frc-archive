package frc3512.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot2022.Constants;

public class Intake extends SubsystemBase {

  public enum IntakeDirection {
    kIdle,
    kIntake,
    kOuttake
  };

  public IntakeDirection m_currState = IntakeDirection.kIdle;

  CANSparkMax m_intakeMotor =
      new CANSparkMax(Constants.Intake.kArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax m_miniArmMotor =
      new CANSparkMax(Constants.Intake.kMiniArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

  Solenoid m_fourbar = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.kArmChannel);

  /** Subsystem class for the intake */
  public Intake() {
    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_miniArmMotor, Usage.kMinimal);
    m_miniArmMotor.setSmartCurrentLimit(80);
    CANSparkMaxUtil.SetCANSparkMaxBusUsage(m_intakeMotor, Usage.kMinimal);
    m_intakeMotor.setSmartCurrentLimit(80);
  }

  public void deploy() {
    m_fourbar.set(true);
  }

  public void stow() {
    m_fourbar.set(false);
  }

  public boolean isDeployed() {
    return m_fourbar.get();
  }

  public void runIntake() {
    m_intakeMotor.set(0.8);
    m_miniArmMotor.set(-0.8);
    m_currState = IntakeDirection.kIntake;
  }

  public void runOuttake() {
    m_intakeMotor.set(-0.8);
    m_miniArmMotor.set(0.8);
    m_currState = IntakeDirection.kOuttake;
  }

  public void runIdle() {
    m_intakeMotor.set(0.0);
    m_miniArmMotor.set(0.0);
    m_currState = IntakeDirection.kIdle;
  }

  public void stopIntake() {
    runIdle();
  }

  @Override
  public void periodic() {}
}
