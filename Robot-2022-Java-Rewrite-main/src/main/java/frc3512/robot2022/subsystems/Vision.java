package frc3512.robot2022.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.NetworkTableUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  public static String kVisionCamName = "mmal_service_16.1";

  public static double kTargetHeight = 2.6;
  public static double kCameraHeight = 0.76835;
  public static double kCameraPitch = 44.6;
  public static double kCameraDiagnoalFOV = 74.8;
  public static double kYawOffset = 3.5;

  PhotonCamera m_visionCamera = new PhotonCamera(kVisionCamName);

  static boolean m_haveTargets = false;

  private static int bufferIndex = 0;

  double m_yaw;
  double m_pitch;
  double m_range;

  static CircularBuffer m_yawBuffer;
  static CircularBuffer m_pitchBuffer;
  static CircularBuffer m_rangeBuffer;

  NetworkTableEntry m_poseEntry =
      NetworkTableUtil.MakeDoubleArrayEntry("/Diagnostics/Vision/Drivetrain Pose");
  NetworkTableEntry m_yawEntry = NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Yaw");
  NetworkTableEntry m_rangeEntry = NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Range");
  NetworkTableEntry m_timestampEntry =
      NetworkTableUtil.MakeDoubleEntry("/Diagnostics/Vision/Timestamp");

  /** Subsystem class for vision */
  public Vision() {}

  public static double getLatestYaw() {
    return m_yawBuffer.get(bufferIndex);
  }

  public static double getLatestPitch() {
    return m_pitchBuffer.get(bufferIndex);
  }

  public static double getLatestRange() {
    return m_rangeBuffer.get(bufferIndex);
  }

  public static boolean hasTargets() {
    return m_haveTargets;
  }

  @Override
  public void periodic() {
    if (!RobotBase.isSimulation()) {
      var result = m_visionCamera.getLatestResult();

      if (result.getTargets().size() == 0 || DriverStation.isDisabled()) {
        m_haveTargets = false;
        return;
      }

      m_haveTargets = true;

      var timestamp = Timer.getFPGATimestamp() - result.getLatencyMillis();

      m_timestampEntry.setDouble(timestamp);

      PhotonTrackedTarget target = result.getBestTarget();

      m_pitch = target.getPitch();
      if (target.getYaw() < 0.0) {
        m_yaw = target.getYaw() - kYawOffset;
      } else if (target.getYaw() > 0.0) {
        m_yaw = target.getYaw() + kYawOffset;
      } else {
        m_yaw = target.getYaw();
      }

      m_yawEntry.setDouble(m_yaw);

      m_range =
          PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight, 2.606, kCameraHeight, m_pitch);

      m_rangeEntry.setDouble(m_range);

      m_yawBuffer.addLast(m_yaw);
      m_pitchBuffer.addLast(m_pitch);
      m_rangeBuffer.addLast(m_range);

      bufferIndex++;
    }
  }
}
