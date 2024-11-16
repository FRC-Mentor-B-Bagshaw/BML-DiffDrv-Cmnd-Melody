// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

//import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera; //https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create Differential Drive Motor Contorllers
  private final Spark m_leftLeader = new Spark(0);
  private final Spark m_rightLeader = new Spark(1);
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
// Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(3, 3);

  
  // Create the Camera and the Pose objects
  PhotonCamera noteCamera = new PhotonCamera("AVerMedia_PW315");
  PhotonCamera tagCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  public static boolean pHasTarget,  tagTarget;
  public Transform3d cameraToRobot = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0, Units.degreesToRadians(15), 0)); 
  public Pose2d robotPosition = new Pose2d();


  // Create the Field Model 
  private final Field2d m_field = new Field2d();
  public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Create Differential Drive Pose Objects
  private final Encoder m_leftEncoder = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
  private final Encoder m_rightEncoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final DifferentialDriveKinematics m_kinematics =
  new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

  private final DifferentialDrivePoseEstimator m_odometry =
  new DifferentialDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kY)* (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  
  //private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
 

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  m_rightEncoder.setSamplesToAverage(5);
  m_leftEncoder.setSamplesToAverage(5);
  m_rightEncoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // pulses averaged from l & r = 12928 of 10' (5 x 2' tiles)
  m_leftEncoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // 0.3048 converts feet into metres
  m_rightEncoder.setMinRate(0.01);
  m_leftEncoder.setMinRate(0.01);
  m_rightEncoder.reset();
  m_leftEncoder.reset();
 
      }
 
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update( 
      Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kY)* (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
      );
    
    var tagResult = tagCamera.getLatestResult();
    tagTarget = tagResult.hasTargets();
    if (tagTarget) {
      PhotonTrackedTarget target = tagResult.getBestTarget();
      Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
      Pose3d pvRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(), 
        tagPose, 
        cameraToRobot);
      Pose2d robotPosition = new Pose2d(pvRobotPose.getTranslation().toTranslation2d(), pvRobotPose.getRotation().toRotation2d());
      m_field.setRobotPose(robotPosition); // change robotPosition to 'm_odometry.getPoseMeters()'
      SmartDashboard.putData("pvrobotPose", m_field);
      m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kY)* (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),robotPosition);
      
    }
    else{
      m_field.setRobotPose(m_odometry.getEstimatedPosition());
      SmartDashboard.putData("pvrobotPose", m_field);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(-m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kY)),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      robotPosition);
  }

   /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);
  }

 /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  
 /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_rightEncoder.reset();
    m_leftEncoder.reset();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY)).getDegrees()* (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kY) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
