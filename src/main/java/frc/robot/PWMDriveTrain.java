package frc.robot;

import java.util.function.Supplier;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.physics.DifferentialDrive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.PWMTrajectoryTrackerCommand;
import frc.robot.lib.EncodedSpark;
import frc.robot.lib.HalfBakedEncodedPWMMotorController;
import frc.robot.lib.PIDSetting;
import frc.robot.lib.drivebases.DifferentialTrackerDriveBase;
import frc.robot.lib.drivebases.DriveTrainBase;

public class PWMDriveTrain extends Subsystem implements DifferentialTrackerDriveBase, DriveTrainBase<HalfBakedEncodedPWMMotorController> {

	EncodedSpark lMaster, rMaster;//, lSlave, rSlave;
	public static final NativeUnitLengthModel leftLengthModel = new NativeUnitLengthModel(RobotConfig.driveTrainUnitsPerRot, RobotConfig.leftWheelRadius);
	public static final NativeUnitLengthModel rightLengthModel = new NativeUnitLengthModel(RobotConfig.driveTrainUnitsPerRot, RobotConfig.rightWheelRadius);
	RamseteTracker ramseteTracker;
	/* Ramsete constants */
	public static final double kDriveBeta = 2 * 1d; // Inverse meters squared
	public static final double kDriveZeta = 0.7 * 1d; // Unitless dampening co-efficient

	Localization localization;

	public Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectSource;

	public AHRS gyro = new AHRS(SPI.Port.kMXP);
	double gyroZero;

	private static PWMDriveTrain inst;

	private DifferentialDrive differentialDrive;

	// public final DCMotorTransmission leftTransmission;// = new DCMotorTransmission(1 / kVDriveLeftLow,
	// kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
	// kVInterceptLeftLow);

	// public final DCMotorTransmission rightTransmission;// = new DCMotorTransmission(1 / kVDriveRightLow,
	// kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
	// kVInterceptRightLow);

	protected PWMDriveTrain() {

		differentialDrive = RobotConfig.VeryStatic.kLowGearDifferentialDrive;

		var leftEncoder = new Encoder(Constants.kLeftEncoderA, Constants.kLeftEncoderB, Constants.kLeftEncoderInvert);
		var leftModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(1024), Constants.kWheelDiameter.div(2));
		var leftPID = new PIDSetting(1, 0, 10, 0);

		var rightEncoder = new Encoder(Constants.kRightEncoderA, Constants.kRightEncoderB, Constants.kRightEncoderInvert);
		var rightModel = new NativeUnitLengthModel(NativeUnitKt.getNativeUnits(1024), Constants.kWheelDiameter.div(2));
		var rightPID = new PIDSetting(1, 0, 10, 0);

		lMaster = new EncodedSpark(Constants.kLeftMotor, leftEncoder, leftModel, leftPID);
		// lSlave = new EncodedSpark(RobotConfig.leftSlaveMotorPort, leftLengthModel);
		// lSlave.set(ControlMode.Follower, lMaster.getDeviceID());

		rMaster = new EncodedSpark(Constants.kRightMotor, rightEncoder, rightModel, rightPID);
		// rSlave = new HalfBakedEncodedPWMMotorController(RobotConfig.rightSlaveMotorPort, rightLengthModel, TimeUnitsKt.getMillisecond(10));
		// rSlave.set(ControlMode.Follower, rMaster.getDeviceID());

		ramseteTracker = new RamseteTracker(kDriveBeta, kDriveZeta);

		/* Create a localization object because lamda expressions are fun */
		localization = new TankEncoderLocalization(() -> getGyro(),
				() -> getLeftDistance(), () -> getRightDistance());

		/* set the robot pose to 0,0,0 */
		localization.reset(new Pose2d());

		this.trajectSource = () -> (Trajectories.generatedHGTrajectories.get("habM to cargoMR"));
	}

	/**
	 * Get the angle of the gyro, accounting for the gyro zero angle
	 * 
	 * @return compensated gyro angle
	 */
	public Rotation2d getGyro() {
		return Rotation2dKt.getDegree(gyro.getAngle() - gyroZero);
	}

	public Length getLeftDistance() {
		return getLeftMotor().getDistance();
	}

	public Length getRightDistance() {
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

	public static PWMDriveTrain getInstance() {
		if (inst == null)
			inst = new PWMDriveTrain();
		return inst;
	}

	@Override
	public void stop() {
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

	@Override
	public void tankDrive(double leftPercent, double rightPercent) {
		//TODO do we want deadbanding?

		getLeftMotor().set(leftPercent);
		getRightMotor().set(rightPercent);
	}

	@Override
	public Subsystem getRealSubsystem() {
		return this;
	}

	@Override
	public void setNeutralMode(NeutralMode mode) {
		//FIXME i guess dont?????
	}

	private void setTrajectorySource(TimedTrajectory<Pose2dWithCurvature> traject) {
		this.trajectSource = () -> traject;
	}

	public PWMTrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory) {
		this.setTrajectorySource(trajectory);
		return new PWMTrajectoryTrackerCommand(this, this.trajectSource);
	}

}
