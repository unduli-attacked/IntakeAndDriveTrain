package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

public class PWMDriveTrain extends Subsystem implements DifferentialTrackerDriveBase {

  EncodedSpark lMaster, rMaster;//, lSlave, rSlave;
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

  private DifferentialDrive differentialDrive;

	private static final double kVDriveLeftLow = 0.265 * 1d; // Volts per radians per second - Calculated emperically
	private static final double kADriveLeftLow = 0.027 * 1d; // Volts per radians per second per second TODO tune
	private static final double kVInterceptLeftLow = 0.95 * 1d; // Volts - tuned!

	private static final double kVDriveRightLow = 0.275 * 1d; // Volts per radians per second - Calculated emperically
	private static final double kADriveRightLow = 0.0286 * 1d; // Volts per radians per second per second TODO tune
	private static final double kVInterceptRightLow = 0.96 * 1d; // Volts - tuned!

	public final DCMotorTransmission leftTransmission;// = new DCMotorTransmission(1 / kVDriveLeftLow,
			// kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
			// kVInterceptLeftLow);

	public final DCMotorTransmission rightTransmission;// = new DCMotorTransmission(1 / kVDriveRightLow,
			// kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
			// kVInterceptRightLow);


  protected PWMDriveTrain(){

    var mass = MassKt.getLb(100);
    var moi = 10;
    var angularDrag = 12;
    var wheelRadius = LengthKt.getInch(6);
    var trackWidth = LengthKt.getInch(24); // todo change me

    leftTransmission = new DCMotorTransmission(1 / kVDriveLeftLow,
    wheelRadius.getMeter() * wheelRadius.getMeter() * mass.getKilogram() / (2.0 * kADriveLeftLow),
    kVInterceptLeftLow);

    rightTransmission = new DCMotorTransmission(1 / kVDriveRightLow,
      wheelRadius.getMeter() * wheelRadius.getMeter() * mass.getKilogram() / (2.0 * kADriveRightLow),
      kVInterceptRightLow);

    differentialDrive = new DifferentialDrive(mass.getKilogram(), moi, angularDrag, wheelRadius.getMeter(), trackWidth.getMeter() / 2, leftTransmission, rightTransmission);

    var leftEncoder = new Encoder(Constants.kLeftEncoderA, Constants.kLeftEncoderB, Constants.kLeftEncoderInvert);
    var leftModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(1024), Constants.kWheelDiameter.div(2));

    lMaster = new EncodedSpark(Constants.kLeftMotor, leftEncoder, model, settings);
    // lSlave = new EncodedSpark(RobotConfig.leftSlaveMotorPort, leftLengthModel);
    // lSlave.set(ControlMode.Follower, lMaster.getDeviceID());

    rMaster = new EncodedSpark(RobotConfig.rightMasterMotorPort, rightLengthModel, TimeUnitsKt.getMillisecond(10));
    // rSlave = new HalfBakedEncodedPWMMotorController(RobotConfig.rightSlaveMotorPort, rightLengthModel, TimeUnitsKt.getMillisecond(10));
    // rSlave.set(ControlMode.Follower, rMaster.getDeviceID());


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
    return getLeftMotor().getDistance();
  }

  public Length getRightDistance(){
    return getRightMotor().getDistance();
  }

  // public FalconSRX<Length> getLeftMotor() {
    // return this.lMaster;
  // }

  // public FalconSRX<Length> getRightMotor() {
    // return this.rMaster;
  // }

  public TrajectoryTracker getTrajectoryTracker() {
    return ramseteTracker;
  }

  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
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
    getLeftMotor().set(0);
    getRightMotor().set(0);
  }

  // @Override
  // void setOutput(TrajectoryTrackerOutput output) {

  // }

  // @Override
  // public HalfBakedEncodedPWMMotorController getLeftMotor() {
    // return lMaster;
  // }

  // @Override
  // public HalfBakedEncodedPWMMotorController getRightMotor() {
    // return rMaster;
  // }

  @Override
  public Pose2d getRobotPosition() {
    return localization.getRobotPosition();
  }

  @Override
  public TrajectoryTracker getTracker() {
    return ramseteTracker;
  }

  @Override
  protected void initDefaultCommand() {

  }

  @Override
  public HalfBakedEncodedPWMMotorController getLeftMotor() {
    return lMaster;
  }

  @Override
  public HalfBakedEncodedPWMMotorController getRightMotor() {
    return rMaster;
  }

  // @Override
  // public void setOutput(TrajectoryTrackerOutput output) {

  // }

}