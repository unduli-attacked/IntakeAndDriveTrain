package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.physics.DifferentialDrive;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.DifferentialTrackerDriveBase;

public class PWMDriveTrain extends DifferentialTrackerDriveBase {

  FalconSRX<Length> lMaster, rMaster, lSlave, rSlave;
  public static final NativeUnitLengthModel leftLengthModel = new NativeUnitLengthModel(RobotConfig.driveTrainUnitsPerRot, RobotConfig.leftWheelRadius);
  public static final NativeUnitLengthModel rightLengthModel = new NativeUnitLengthModel(RobotConfig.driveTrainUnitsPerRot, RobotConfig.rightWheelRadius);
  RamseteTracker ramseteTracker;
  	/* Ramsete constants */
	public static final double kDriveBeta = 2 * 1d; // Inverse meters squared
  public static final double kDriveZeta = 0.7 * 1d; // Unitless dampening co-efficient
  
  Localization localization;

  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  double gyroZero;
  
  private static DriveTrain inst;


  protected PWMDriveTrain(){
    lMaster = new FalconSRX<Length>(RobotConfig.leftMasterMotorPort, leftLengthModel, TimeUnitsKt.getMillisecond(10));
    lSlave = new FalconSRX<Length>(RobotConfig.leftSlaveMotorPort, leftLengthModel, TimeUnitsKt.getMillisecond(10));
    lSlave.set(ControlMode.Follower, lMaster.getDeviceID());

    rMaster = new FalconSRX<Length>(RobotConfig.rightMasterMotorPort, rightLengthModel, TimeUnitsKt.getMillisecond(10));
    rSlave = new FalconSRX<Length>(RobotConfig.rightSlaveMotorPort, rightLengthModel, TimeUnitsKt.getMillisecond(10));
    rSlave.set(ControlMode.Follower, rMaster.getDeviceID());


    ramseteTracker = new RamseteTracker(kDriveBeta, kDriveZeta);

    
		/* Create a localization object because lamda expressions are fun */
		localization = new TankEncoderLocalization(() -> getGyro(),
    () -> getLeftDistance(), () -> getRightDistance());

    /* set the robot pose to 0,0,0 */
    localization.reset(new Pose2d());
  }

  /**
	 * Get the angle of the gyro, accounting for the gyro zero angle
	 * 
	 * @return compensated gyro angle
	 */
	public Rotation2d getGyro() {
		return Rotation2dKt.getDegree(gyro.getAngle() - gyroZero);
  }
  
  public Length getLeftDistance(){
    return getLeftMotor().getSensorPosition();
  }

  public Length getRightDistance(){
    return getRightMotor().getSensorPosition();
  }

  public FalconSRX<Length> getLeftMotor() {
    return this.lMaster;
  }

  public FalconSRX<Length> getRightMotor() {
    return this.rMaster;
  }

  public TrajectoryTracker getTrajectoryTracker() {
    return ramseteTracker;
  }

  public DifferentialDrive getDifferentialDrive() {
    return null;
  }

  public Localization getLocalization() {
    return localization;
  }

  public static DriveTrain getInstance() {
    if(inst == null)
      inst = new DriveTrain();
    return inst;
  }

  public void stop(){
    getLeftMotor().set(ControlMode.PercentOutput, 0);
    getRightMotor().set(ControlMode.PercentOutput, 0);
  }

  @Override
  void setOutput(TrajectoryTrackerOutput output) {

  }

}