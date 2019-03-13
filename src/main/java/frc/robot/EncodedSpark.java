package frc.robot;

import org.ghrobotics.lib.mathematics.units.derivedunits.Volt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VoltKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Encoder;

public class EncodedSpark extends HalfBakedEncodedPWMMotorController {

	public EncodedSpark(int channel, Encoder encoder, NativeUnitLengthModel model, PIDSetting settings) {

		super(channel, encoder, model, settings);

		setBounds(2.003, 1.55, 1.50, 1.46, .999);
		setPeriodMultiplier(PeriodMultiplier.k1X);
		setSpeed(0.0);
		setZeroLatch();

		HAL.report(tResourceType.kResourceType_RevSPARK, getChannel());
		setName("Spark", getChannel());

	}

	@Override
	public double getPercentOutput() {
		return super.get();
	}

	@Override
	public Volt getVoltageOutput() {
		// return getVoltageOutput();
		return VoltKt.getVolt(getPercentOutput() / 12);
	}

}
