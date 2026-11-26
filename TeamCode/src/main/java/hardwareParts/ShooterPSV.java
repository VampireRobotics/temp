package hardwareParts;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Iterator;

@Config
public class ShooterPSV {

    // ===== Hardware =====
    public final DcMotorEx motorShooter;
    public final DcMotorEx motorShootermaster;
    public final Servo unghi;
    public final Telemetry telemetry;
    private final CalculateTargetRPM rpmCalc;

    // ===== Battery =====
    private VoltageSensor batterySensor;
    public double batteryVoltage = 0.0;

    // ===== Velocity constants =====
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_GEAR_RATIO = 1.0;

    // ===== PSV Constants (tunable via Dashboard) =====
    public static double kS = 0.05;  // offset static (putere minima la pornire)
    public static double kV = 0.9;   // feedforward: kS + kV*(RPM/MAX_RPM) = putere motor
    public static double kP = 0.3;   // proportional: corecteaza eroarea reziduala

    // minim ca să nu pice la 0 și să "moară" roata
    public static double MIN_CMD = 0.05;

    // ===== Constants =====
    private static final double VEL_ALPHA = 0.1;
    public static final double COAST_POWER = 0.8;
    public static double CHARLIE_KIRK = 4100; // MAX RPM TARGET
    public static double ANGLE_ADJUST = 0.05;
    public static double angleAdjustCircle = 0.030;

    // ===== Unghi per zona distanta (tunable din Dashboard) =====
    public static double ANGLE_NEAR     = 0.8;   // distanta < 44in
    public static double ANGLE_MID      = 0.37;  // distanta 44–108in, in banda
    public static double ANGLE_MID_MAX  = 0.39;   // limita superioara la ajustare in 44–108in
    public static double ANGLE_FAR      = 0.05;  // distanta > 108in

    private final double DIDDY = 100; // RPM THRESHOLD

    // ===== State =====
    public boolean enabled = true;
    public boolean on = false;

    public double rawVelocityRPM = 0;
    public double wheelVelocityRPM = 0;

    public double targetRPM = 500;
    public double targetAngle = 0;
    public static double startanglecicle = 0;
    public double POWER = 0;
    public double TARGET = 0; // target RPM

    // ===== Servo helpers =====
    public static double SERVO_STEP = 0.05;
    public static final double SERVO_MIN = 0.35;
    public static final double SERVO_MIN_HIGH = 0.2;
    public static final double SERVO_MAX = 0.9;

    // ===== auto-angle adjust =====
    private boolean autoEnabled = false;
    private boolean autoCircleAngle = false;
    private double servoPos = 0.9;
    private double baseAngle = 0.0;

    public int selectedMode = 0;
    public boolean locked = false;

    public int pipeline = 0;
    public int targetID = 20;
    private boolean dpadLeftPrev = false, dpadRightPrev = false, aPrev = false;

    // ===== DEBUG =====
    public double dbg_ty = 0;
    public double dbg_angle = 0;
    public double dbg_distRaw = 0;
    public double dbg_dist = 0;
    public double dbg_servoTarget = 0;
    public boolean dbg_hasTag = false;
    public boolean dbg_sweep = false;

    // distance (from calcRPM)
    public double distance = 0.0;

    // ============================================================
    // Burst logic
    // ============================================================
    private int shotCount = 0;
    public static double DROP_AFTER_2 = 0.03;

    private boolean burstActive = false;
    private boolean droppedThisBurst = false;
    private double burstHomeAngle = 0;

    // ============================================================
    // Voltage compensation
    // ============================================================
    public static double V_MIN = 12;
    public static double V_MAX = 14.4;
    public static double DROP_MAX_AT_130 = 0;
    public static double DIST_MAX_FOR_DROP = 140.0;

    // ============================================================
    // DROP logic
    // ============================================================
    private boolean prevInBand = false;
    private long dropUntilMs = 0;
    private long lastDropMs = 0;

    public static long DROP_TIME_MS = 85;
    public static long DROP_COOLDOWN_MS = 1;
    public static double DROP_AMOUNT = 0.03;
    public static double DROP_THRESHOLD = 100.0; // RPM threshold
    public static double DROP_MIN_TARGET_RPM = 200;

    public static int DROP_SIGN = +1;

    public ShooterPSV(HardwareMap hw, Telemetry tel, CalculateTargetRPM calc) {
        telemetry = tel;
        rpmCalc = calc;

        motorShooter = hw.get(DcMotorEx.class, "motorShooter");
        motorShootermaster = hw.get(DcMotorEx.class, "motorShootermaster");
        unghi = hw.get(Servo.class, "unghi");

        motorShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShootermaster.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShootermaster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShootermaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorShootermaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        batterySensor = it.hasNext() ? it.next() : null;

        setUnghi(servoPos);
        targetAngle = servoPos;

        baseAngle = targetAngle;
    }

    // ===== ready() =====
    public boolean ready() { return ready(DIDDY); }

    public boolean ready(double threshold) {
        return Math.abs(this.TARGET - wheelVelocityRPM) < threshold;
    }

    // ===== read() =====
    public void read() {
        double ticksPerSec = motorShootermaster.getVelocity(); // ticks/sec (raw, fara AngleUnit)
        rawVelocityRPM = ticksPerSec / MOTOR_TICKS_PER_REV * 60.0 / MOTOR_GEAR_RATIO;
        wheelVelocityRPM += VEL_ALPHA * (rawVelocityRPM - wheelVelocityRPM);
    }

    // ===== enable/disable auto-angle =====
    public void enableAutoAngle(boolean en) { autoEnabled = en; }
    public void autocircleAngle(boolean en) { autoCircleAngle = en; }

    // ===== burst control =====
    public void beginShootingBurst() {
        if (burstActive) return;
        burstActive = true;

        droppedThisBurst = false;
        shotCount = 0;

        burstHomeAngle = getUnghi();
    }

    public void endShootingBurst() {
        if (!burstActive) return;

        setUnghi(burstHomeAngle);

        shotCount = 0;
        droppedThisBurst = false;
        burstActive = false;
    }

    public void registerShotInBurst() {
        if (!burstActive) beginShootingBurst();

        shotCount++;

        if (!droppedThisBurst && shotCount >= 2) {
            setUnghi(burstHomeAngle + SERVO_STEP);
            droppedThisBurst = true;
        }
    }

    public void resetShotCounter() {
        shotCount = 0;
        droppedThisBurst = false;
    }

    // ===== battery helper =====
    private double readBatteryVoltage() {
        if (batterySensor == null) return 0.0;
        double v = batterySensor.getVoltage();
        if (v < 1.0) return 0.0;
        return v;
    }

    // ===== voltage + distance drop helper =====
    private double computeVoltageDrop() {
        double tV = (batteryVoltage - V_MIN) / (V_MAX - V_MIN);
        tV = Range.clip(tV, 0.0, 1.0);

        double tD = (DIST_MAX_FOR_DROP <= 0.0) ? 0.0 : (distance / DIST_MAX_FOR_DROP);
        tD = Range.clip(tD, 0.0, 1.0);

        return DROP_MAX_AT_130 * tV * tD;
    }

    // ===== update() - PSV calculation =====
    public void update() {
        if (!enabled) return;

        if (!on) {
            prevInBand = false;
            dropUntilMs = 0;

            motorShooter.setPower(0);
            motorShootermaster.setPower(0);
            return;
        }

        distance = (rpmCalc != null) ? rpmCalc.getDistance() : 0.0;
        batteryVoltage = readBatteryVoltage();

        // PSV control: cmd = kS + kV * targetRPM + kP * error
        this.TARGET = targetRPM;
        double error = TARGET - wheelVelocityRPM;
        double errorNorm = error / MOTOR_MAX_RPM;

        // Feedforward: kS (static friction) + kV * velocity
        double ff = kS + kV * (TARGET / MOTOR_MAX_RPM);

        // Feedback: kP * error
        double fb = kP * errorNorm;

        double cmd = ff + fb;
        this.POWER = Range.clip(cmd, 0, 1.0);

        // Drop logic based on RPM error
        if (targetRPM < DROP_MIN_TARGET_RPM) {
            prevInBand = false;
        } else {
            long now = System.currentTimeMillis();

            double errSmooth = TARGET - wheelVelocityRPM;
            double errFast = TARGET - rawVelocityRPM;

            boolean wasInBand = Math.abs(errSmooth) < DROP_THRESHOLD;
            boolean fastOut = Math.abs(errFast) > DROP_THRESHOLD;

            if (wasInBand && fastOut && (now - lastDropMs) > DROP_COOLDOWN_MS) {
                dropUntilMs = now + DROP_TIME_MS;
                lastDropMs = now;
            }

            prevInBand = wasInBand;
        }

        // ===== Auto-angle logic =====
        if (autoEnabled) {
            if (distance < 44) {
                targetAngle = ANGLE_NEAR;
            } else if (distance >= 44 && distance < 108) {
                double errAngle = TARGET - wheelVelocityRPM;

                if (Math.abs(errAngle) < DIDDY) {
                    targetAngle = ANGLE_MID;
                } else {
                    targetAngle = clamp(
                            targetAngle + errAngle * ANGLE_ADJUST / 100.0,
                            ANGLE_MID,
                            ANGLE_MID_MAX
                    );
                }
            } else {
                targetAngle = ANGLE_FAR;
            }
        }

        if (autoCircleAngle) {
            double errAngle = TARGET - wheelVelocityRPM;

            if (Math.abs(errAngle) < DIDDY) {
                targetAngle = startanglecicle;
            } else {
                targetAngle = clamp(
                        targetAngle + errAngle * angleAdjustCircle / 100.0,
                        SERVO_MIN,
                        SERVO_MAX
                );
            }
        }

        baseAngle = targetAngle;

        // Apply drop override
        if (System.currentTimeMillis() < dropUntilMs) {
            double signed = (DROP_SIGN >= 0) ? DROP_AMOUNT : -DROP_AMOUNT;
            targetAngle = clamp(baseAngle + signed, SERVO_MIN, SERVO_MAX);
        } else {
            targetAngle = baseAngle;
        }
    }

    // ===== write() =====
    public void write() {
        if (!enabled) return;

        if (!on) {
            motorShooter.setPower(0);
            motorShootermaster.setPower(0);
            return;
        }

        double outPower = POWER - computeVoltageDrop();
        outPower = Range.clip(outPower, MIN_CMD, 1.0);

        motorShooter.setPower(outPower);
        motorShootermaster.setPower(outPower);

        unghi.setPosition(targetAngle);
        servoPos = targetAngle;
    }

    // ===== on/off =====
    public void on() {
        enabled = true;
        on = true;

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShootermaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prevInBand = ready(DROP_THRESHOLD);

        dropUntilMs = 0;
        lastDropMs = 0;

        baseAngle = targetAngle;
    }

    public void off() {
        on = false;
        motorShootermaster.setPower(0);
        motorShooter.setPower(0);

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorShootermaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetRPM = 0.0;
        POWER = 0.0;

        prevInBand = false;
        dropUntilMs = 0;

        baseAngle = targetAngle;
    }

    // ===== setEnabled =====
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (enabled) return;

        motorShooter.setMotorDisable();
        motorShootermaster.setMotorDisable();
    }

    // ===== API for teleop =====
    public void setTargetRPM(double rpm) {
        targetRPM = clamp(rpm, 0.0, CHARLIE_KIRK);
    }

    public void setMotorPower(double power) {
        motorShootermaster.setPower(power);
        motorShooter.setPower(power);
    }

    public double getTicksPerSec() {
        return motorShootermaster.getVelocity();
    }

    public double getRPM() {
        return ((getTicksPerSec() / MOTOR_TICKS_PER_REV) * 60) / MOTOR_GEAR_RATIO;
    }

    public void setTargetAngle(double angle01) {
        targetAngle = clamp(angle01, SERVO_MIN, SERVO_MAX);
        baseAngle = targetAngle;
    }

    // ===== Servo helpers =====
    public void setUnghi(double pos) {
        servoPos = Range.clip(pos, SERVO_MIN, SERVO_MAX);
        unghi.setPosition(servoPos);
        targetAngle = servoPos;
        baseAngle = targetAngle;
    }

    public double getUnghi() { return servoPos; }

    public void unghi_sus() { setUnghi(getUnghi() - SERVO_STEP); }
    public void unghi_jos() { setUnghi(getUnghi() + SERVO_STEP); }
    public void poz0() { setUnghi(SERVO_MIN); }

    // ===== init =====
    public void init() {
        setUnghi(servoPos);
        telemetry.addLine("Init:  select BLUE/RED (DPAD) + LOCK (CROSS).");
        setTargetRPM(0.0);
        on = false;

        burstActive = false;
        droppedThisBurst = false;
        shotCount = 0;
        burstHomeAngle = getUnghi();

        prevInBand = false;
        dropUntilMs = 0;
        lastDropMs = 0;
    }

    // ===== telemetry =====
    public void printTelemetry() {
        telemetry.addData("[ShooterPSV] On", on);
        telemetry.addData("[ShooterPSV] TargetRPM", "%.0f", TARGET);
        telemetry.addData("[ShooterPSV] ActualRPM", "%.0f", wheelVelocityRPM);
        telemetry.addData("[ShooterPSV] Error", "%.0f", TARGET - wheelVelocityRPM);
        telemetry.addData("[ShooterPSV] Power", "%.3f", POWER);
        telemetry.addData("[ShooterPSV] Angle", "%.3f", targetAngle);
        telemetry.addData("[ShooterPSV] Distance", "%.1f", distance);
        telemetry.addData("[ShooterPSV] Battery", "%.2f", batteryVoltage);
    }
}
