package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitVelocityKt;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Spark;

public abstract class HalfBakedEncodedPWMMotorController extends PWMSpeedController {

    private Encoder encoder;
    private NativeUnitLengthModel model;
    private PIDController controller;
    private double arbitraryFeedForwardValue = 0;

    public HalfBakedEncodedPWMMotorController(int channel, Encoder encoder, NativeUnitLengthModel model, PIDSetting settings) {
        super(channel);
        this.encoder = encoder;
        this.model = model;
        encoder.setPIDSourceType(PIDSourceType.kRate);
        controller = new PIDController(settings.kp, settings.ki, settings.kd, encoder, this);
    }

    public void setVelocityAndArbitraryFeedForward(Velocity<Length> speed, double arbitraryFeedForward) {
        var rawVel = model.toNativeUnitVelocity(speed);

        controller.setSetpoint(rawVel.getValue());

        arbitraryFeedForwardValue = arbitraryFeedForward;
    }

    public Length getDistance() {
        var rawDistance = encoder.getDistance();
        var distance = model.fromNativeUnitPosition(NativeUnitKt.getNativeUnits(rawDistance));
        return distance;
    }

    public double getRawPos() {
        return encoder.pidGet();
    }

    @Override
    public void pidWrite(double output) {
        set(output + arbitraryFeedForwardValue);
    }

}