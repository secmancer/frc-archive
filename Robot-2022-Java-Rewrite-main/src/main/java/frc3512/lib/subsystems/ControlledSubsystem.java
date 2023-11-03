package frc3512.lib.subsystems;

import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot2022.GeneralConfig;

/** Subystem class that has timestep updating abilities for state-space controllers */
public class ControlledSubsystem<States extends Num, Inputs extends Num, Outputs extends Num>
    extends SubsystemBase {

  double nowBegin = Timer.getFPGATimestamp();
  double lastTime = Timer.getFPGATimestamp();
  double dt = GeneralConfig.kControllerPeriod;
  boolean isEnabled = false;

  public ControlledSubsystem() {
    DataLogManager.start();
  }

  public void enable() {
    lastTime = Timer.getFPGATimestamp() - GeneralConfig.kControllerPeriod;
    isEnabled = true;
  }

  public void disable() {
    isEnabled = false;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  public double getDT() {
    return dt;
  }

  public void controllerPeriodic() {}

  public void updateDT() {
    nowBegin = Timer.getFPGATimestamp();
    dt = nowBegin - lastTime;

    if (dt == 0.0) {
      dt = GeneralConfig.kControllerPeriod;
      System.err.printf("Error: @ t = %d: dt = 0\n", nowBegin);
    }

    if (dt > 10.0) {
      dt = GeneralConfig.kControllerPeriod;
    }
  }

  @Override
  public void periodic() {}
}
