package frc.robot.subsystems.base;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;  //was can sparkmax
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.TunableNumber;
import frc.robot.util.pid.TunablePID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;; //CANSparkLowLevel.MotorType.kBrushless

public class BaseSubsystem extends SubsystemBase {
    private final ArrayList<TunableNumber> numberTunes;
    private final ArrayList<TunablePID> pidTunes;
    private final String name;
    private final SparkMax[] motors;
    private final boolean enabled;

    private long nextUpdate = System.currentTimeMillis();
    private Runnable dashUpdate = () -> {};
    private boolean tuningEnabled;

    public static ArrayList<String> initSystems = new ArrayList<>();

    public void setTuningEnabled(boolean enabled) { this.tuningEnabled = enabled; }
    public boolean isEnabled() { return this.enabled; }

    public void registerConstant(String constantName, double value) {
        TunableNumber tune = new TunableNumber(
                this.name + "/" + constantName,
                value,
                tuningEnabled
        );
        numberTunes.add(tune);
    }

    public PIDController registerPID(String name, PIDConstants constants) {
        String pidName = this.name + "/" + name;
        TunablePID tune = new TunablePID(pidName, constants, tuningEnabled);//PROBLEM
        PIDController controller = GlobalUtils.generateController(constants); //PROBLEM
        tune.addConsumer(controller::setP, controller::setI, controller::setD);

        pidTunes.add(tune);
        return controller;
    }

    @SuppressWarnings("UnusedReturnValue")
    public boolean syncDashboardPID(String name, PIDController controller) {
        for (TunablePID pid : pidTunes) {
            if (pid.getName().equalsIgnoreCase(this.name + "/" + name)) {
                pid.setPID(controller.getP(), controller.getI(), controller.getD());
                return true;
            }
        }
        return false;
    }

    public double getConstant(String name) { return getConstant(name, 0); }
    public double getConstant(String name, double defaultValue) {
        for (TunableNumber number : numberTunes) {
            if (number.getName().equalsIgnoreCase(this.name + "/" + name)) {
                return number.getValue();
            }
        }
        return defaultValue;
    }

    @SuppressWarnings("UnusedReturnValue")
    public boolean setConstant(String name, double value) {
        for (TunableNumber number : numberTunes) {
            if (number.getName().equalsIgnoreCase(this.name + "/" + name)) {
                number.setValue(value);
                return true;
            }
        }
        return false;
    }

    public BaseSubsystem(SubsystemConfig config,
                         Map<Integer, Boolean> ids) {

        this.name = config.name();
        this.enabled = config.enabled();
        this.tuningEnabled = config.tuningEnabled();

        this.motors = new CANSparkMax[ids.size()];
        this.numberTunes = new ArrayList<>();
        this.pidTunes = new ArrayList<>();

        if (initSystems.contains(name)) {
            // Do not double-initialize!
            DriverStation.reportWarning("Subsystem: " + name + " attempted re-initialization!", false);
            return;
        }

        if (!ids.isEmpty() && enabled) {
            AtomicInteger i = new AtomicInteger(0);
            ids.forEach((id, flip) -> {
                motors[i.get()] = new SparkMax(id, kBrushless);
                motors[i.get()].setInverted(flip);
                motors[i.get()].setIdleMode(kBrake);
                motors[i.get()].setSmartCurrentLimit(40); //PROBLEM
                GlobalUtils.executeWithDelay(() -> motors[i.get()].burnFlash(), 2000); //PROBLEM
                i.set(i.get() + 1);
            });
        }
    }

    public void setDashUpdate(Runnable runnable) { this.dashUpdate = runnable; }

    public double getRPM() {
        if (motors.length == 0)
            return 0;
        double sum = 0;
        for (SparkMax motor : motors) {
            if (motor == null)
                continue;
            RelativeEncoder encoder = motor.getEncoder();
            sum += Math.abs(encoder.getVelocity());
        }
        return sum / motors.length;
    }

    public boolean isTuningEnabled() { return this.tuningEnabled; }

    public void stop() {
        for (SparkMax motor : motors) {
            if (motor == null)
                continue;
            motor.stopMotor();
        }
    }

    protected void startAll(double speed) {
        for (SparkMax motor : motors) {
            if (motor == null)
                continue;
            motor.set(speed);
        }
    }

    protected void stopAll() { startAll(0); }

    @SuppressWarnings("UnusedReturnValue")
    protected boolean setIndex(int idx, double speed) {
        if (idx > motors.length-1)
            return false;
        if (motors[idx] == null)
            return true;
        motors[idx].set(speed);
        return true;
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;
        if (System.currentTimeMillis() >= nextUpdate) {
            if (tuningEnabled) {
                for (TunableNumber tune : numberTunes)
                    tune.update();
                for (TunablePID pidTune : pidTunes)
                    pidTune.update();
                for (int i = 0; i < motors.length; i++) {
                    if (motors[i] == null)
                        continue;
                    SmartDashboard.putNumber(name + "/Motor " + i + " Amps", motors[i].getOutputCurrent());
                }
            }
            if (dashUpdate != null)
                dashUpdate.run();

            nextUpdate = System.currentTimeMillis() + 1000;
        }
    }
}
