package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.FalconMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMSpeedController;
import frc.robot.Robot.Logger;

public abstract class HalfBakedEncodedPWMMotorController extends PWMSpeedController implements FalconMotor<Length> {

	private Encoder encoder;
	private NativeUnitLengthModel model;
	private PIDController controller;
	private double arbitraryFeedForwardValue = 0;

	public HalfBakedEncodedPWMMotorController(int channel, Encoder encoder, NativeUnitLengthModel model, PIDSetting settings) {
		super(channel);
		this.encoder = encoder;
		this.model = model;
		encoder.setPIDSourceType(PIDSourceType.kRate);
		controller = new PIDController(settings.kp, settings.ki, settings.kd, encoder, this, 0.01);

		controller.enable();
	}

	public synchronized void setVelocity(Velocity<Length> speed) {
		setVelocityAndArbitraryFeedForward(speed, 0);
	}

	public synchronized void setVelocityAndArbitraryFeedForward(Velocity<Length> speed, double arbitraryFeedForward) {

		if(!controller.isEnabled()) controller.enable();

		// Logger.log("setting to speed and arb ff");

		var rawVel = model.toNativeUnitVelocity(speed);

		// Logger.log("raw vel is " + rawVel.getValue() + " with arb ff of " + arbitraryFeedForward);

		controller.setSetpoint(rawVel.getValue());



		// Logger.log("controller setpoint is " + controller.getSetpoint());

		Logger.log("controller err is " + controller.getError());

		Logger.log("controller is enabled? " + controller.isEnabled());

		arbitraryFeedForwardValue = arbitraryFeedForward;
	}

	public synchronized void setPercentOutputArbFF(double percent, double arbFF) {
		this.arbitraryFeedForwardValue = arbFF;
		if(controller.isEnabled()) controller.disable();
		set(percent + arbFF);
	}

	public synchronized void setPercentOutput(double demand) {
		setPercentOutputArbFF(demand, 0);
	}

	public Length getDistance() {
		var rawDistance = encoder.getDistance();
		var distance = model.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(rawDistance));
		return distance;
	}

	@Override
	public Velocity<Length> getVelocity() {
		double ticks = encoder.getRate();
		var velocity = model.fromNativeUnitVelocity(VelocityKt.getVelocity(NativeUnitKt.getNativeUnits(ticks)));
		return velocity;
	}

	public double getRawPos() {
		return encoder.pidGet();
	}

	@Override
	public synchronized void pidWrite(double output) {
		// System.out.println("setting pwm controller to " + (output + arbitraryFeedForwardValue));
		set(output + arbitraryFeedForwardValue);
	}

}
