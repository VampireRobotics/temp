package hardwareParts;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Iterator;

@Config
public class ShooterPIDPower {

    // ===== Hardware =====
    public final DcMotorEx motorShooter;        // Shooter1
    public final DcMotorEx motorShootermaster;  // Shooter2 (encoder feedback)
    public final Servo unghi;
    public final Telemetry telemetry;
    private final CalculateTargetPower distanceCalc;

    // ===== Battery =====
    private VoltageSensor batterySensor;
    public double batteryVoltage = 0.0;

    // ===== Velocity constants =====
    public static double MOTOR_TICKS_PER_REV = 28;   // goBILDA
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_GEAR_RATIO = 1.0;

    // ===== PIDF =====
    public static double kP = 0.003;//0.008; 0.05
    public static double kI = 0.0;// 0.0007
    public static double kD = 0.0;//0.0002;
    public static double kF = 10;// 0.00329; 17

    public double kV, kS;

    // minim ca să nu pice la 0 și să “moară” roata
    public static double MIN_CMD = 0.05;

    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    // ===== Constants =====
    private static final double VEL_ALPHA = 0.1; //0.3
    public static final double COAST_POWER = 0.8;
    public static double CHARLIE_KIRK = 300; // MAX VELOCITY //272
    public static double ANGLE_ADJUST = 0.05;
    public static double angleAdjustCircle = 0.030;

    private final double DIDDY = 2; // THRESHOLD

    // ===== State =====
    public boolean enabled = true;
    public boolean on = false;

    public double rawVelocity = 0;     // deg/s
    public double wheelVelocity = 0;   // filtered deg/s

    public double targetPower = 0.1;     // 0..1
    public double targetAngle = 0;
    public static double startanglecicle = 0; // 0..1 servo
    public double POWER = 0;           // motor power cmd (PID output)
    public double TARGET = 0;          // velocity target

    // ===== Servo helpers =====
    public static double SERVO_STEP = 0.05;
    public static final double SERVO_MIN = 0.35;
    public static final double SERVO_MIN_HIGH=0.2;
    public static final double SERVO_MAX = 0.9;

    // ===== auto-angle adjust =====
    private boolean autoEnabled = false;
    private boolean autoCircleAngle = false;
    private double servoPos = 0.9;

    // >>> ADDED: păstrăm unghiul de bază (auto/manual), ca drop-ul să fie doar override temporar
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

    // distance (from calcPower)
    public double distance = 0.0;

    // ============================================================
    //  Burst logic (poate rămâne, dar NU îl folosim pt drop)
    // ============================================================
    private int shotCount = 0;
    public static double DROP_AFTER_2 = 0.03;

    private boolean burstActive = false;
    private boolean droppedThisBurst = false;
    private double burstHomeAngle = 0;

    // ============================================================
    //  Voltage compensation (LINEAR) scaled by distance
    // ============================================================
    public static double V_MIN = 12;
    public static double V_MAX = 14.4;
    public static double DROP_MAX_AT_130 = 0;
    public static double DIST_MAX_FOR_DROP = 140.0; // inches

    // ============================================================
    //  DROP pe bază de eroare: declanșează la ÎNCEPUTUL dip-ului
    //  (ieșire din bandă) + cooldown + timer
    // ============================================================
    private boolean prevInBand = false;
    private long dropUntilMs = 0;
    private long lastDropMs = 0;

    public static long DROP_TIME_MS = 85; //180          // cât timp ține drop-ul
    public static long DROP_COOLDOWN_MS = 1;//350      // pauză între drop-uri
    public static double DROP_AMOUNT = 0.03;        // offset servo (0..1)
    public static double DROP_THRESHOLD = 2.0;      // threshold pe eroare (deg/s)
    public static double DROP_MIN_TARGET_POWER = 0.08; // nu declanșa când e aproape OFF

    // Dacă “coboară” la tine înseamnă să scazi poziția, pune -1.
    // Dacă “coboară” înseamnă să crești poziția, lasă +1.
    public static int DROP_SIGN = +1;

    public ShooterPIDPower(HardwareMap hw, Telemetry tel, CalculateTargetPower calc) {
        telemetry = tel;
        distanceCalc = calc;

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

        // battery sensor (pick first available)
        Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        batterySensor = it.hasNext() ? it.next() : null;

        // servo init
        setUnghi(servoPos);
        targetAngle = servoPos;

        // >>> ADDED: inițializăm baza
        baseAngle = targetAngle;
    }

    // ===== ready() =====
    public boolean ready() { return ready(DIDDY); }

    public boolean ready(double DIDDY) {
        return Math.abs(this.TARGET - wheelVelocity) < DIDDY;
    }

    // ===== read() =====
    public void read() {
        rawVelocity = motorShootermaster.getVelocity(AngleUnit.DEGREES);
        wheelVelocity += VEL_ALPHA * (rawVelocity - wheelVelocity);
    }

    // ===== enable/disable auto-angle adjust =====
    public void enableAutoAngle(boolean en) { autoEnabled = en; }
    public void autocircleAngle(boolean en) { autoCircleAngle = en; }

    // ===== burst control (nefolosit pt drop acum) =====
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

        if (!droppedThisBurst && shotCount >=2) {
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
        // voltage factor [0..1]
        double tV = (batteryVoltage - V_MIN) / (V_MAX - V_MIN);
        tV = Range.clip(tV, 0.0, 1.0);

        // distance factor [0..1]
        double tD = (DIST_MAX_FOR_DROP <= 0.0) ? 0.0 : (distance / DIST_MAX_FOR_DROP);
        tD = Range.clip(tD, 0.0, 1.0);

        return DROP_MAX_AT_130 * tV * tD;
    }

    // ===== update() =====
    public void update() {
        if (!enabled) return;

        if (!on) {
            // reset drop edge detect când e OFF (altfel poate declanșa aiurea la ON)
            prevInBand = false;
            dropUntilMs = 0;

            motorShooter.setPower(0);
            motorShootermaster.setPower(0);
            return;
        }

        // read distance + battery each loop
        distance = (distanceCalc != null) ? distanceCalc.getDistance() : 0.0;
        batteryVoltage = readBatteryVoltage();

        controller.setPIDF(kP, kI, kD, 0.0);

        this.TARGET = targetPower * CHARLIE_KIRK;
        double pid = controller.calculate(wheelVelocity, TARGET);
        double ff = targetPower;
        double cmd = ff + pid;
        this.POWER = Range.clip(cmd, 0, 1.0);

        if (targetPower < DROP_MIN_TARGET_POWER) {
            prevInBand = false;
        } else {
            long now = System.currentTimeMillis();

            double errSmooth = TARGET - wheelVelocity;  // stabil
            double errFast   = TARGET - rawVelocity;    // rapid

            boolean wasInBand = Math.abs(errSmooth) < DROP_THRESHOLD;  // condiție stabilă
            boolean fastOut   = Math.abs(errFast)   > DROP_THRESHOLD;  // început dip instant

            if (wasInBand && fastOut && (now - lastDropMs) > DROP_COOLDOWN_MS) {
                dropUntilMs = now + DROP_TIME_MS;
                lastDropMs = now;
            }

            prevInBand = wasInBand;
        }

        // ===== Auto-angle logic (cum aveai) =====
        if (autoEnabled) {
            // prioritate: distanță mică => unghi fix imediat
            if (distance < 44) {
                targetAngle = 0.9;   // ce vrei tu
            } else if (distance>=44 && distance<108) {
                double error = TARGET - wheelVelocity;

                if (Math.abs(error) < DIDDY) {
                    targetAngle = 0.25;
                } else {
                    targetAngle = clamp(
                            targetAngle + error * ANGLE_ADJUST,
                            0.25,
                            0.4
                    );
                }
            }
            else{
                targetAngle = 0.25;
            }
        }

        if (autoCircleAngle) {
            double error = TARGET - wheelVelocity;

            if (Math.abs(error) < DIDDY) {
                targetAngle = startanglecicle;
            } else {
                targetAngle = clamp(
                        targetAngle + error * angleAdjustCircle,
                        SERVO_MIN,
                        SERVO_MAX
                );
            }
        }

        // >>> ADDED: salvăm unghiul de bază (auto/manual) ÎNAINTE de drop
        baseAngle = targetAngle;

        // ===== APPLY DROP OVERRIDE LA FINAL (NU mai “derivă” unghiul) =====
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

        // dacă vrei minim:
        outPower = Range.clip(outPower, MIN_CMD, 1.0);

        motorShooter.setPower(outPower);
        motorShootermaster.setPower(outPower);

        unghi.setPosition(targetAngle);
        servoPos = targetAngle;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60.0;
    }

    public static double ticksPerSecondToRPM(double ticks) {
        return ticks * 60.0 * MOTOR_GEAR_RATIO / MOTOR_TICKS_PER_REV;
    }

    public void sortatorcomp() {}

    // ===== on/off =====
    public void on() {
        enabled = true;
        on = true;

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShootermaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: initializează prevInBand ca să detecteze primul dip corect
        prevInBand = ready(DROP_THRESHOLD);

        dropUntilMs = 0;
        lastDropMs = 0;

        // >>> ADDED: baze corecte la pornire
        baseAngle = targetAngle;
    }

    public void off() {
        on = false;
        motorShootermaster.setPower(0);
        motorShooter.setPower(0);

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorShootermaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        targetPower = 0.0;
        POWER = 0.0;

        prevInBand = false;
        dropUntilMs = 0;

        // >>> ADDED
        baseAngle = targetAngle;
    }

    // ===== setEnabled =====
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (enabled) return;

        motorShooter.setMotorDisable();
        motorShootermaster.setMotorDisable();
    }

    public void setPIDF(double p, double i, double d, double f) {
        controller.setPIDF(p, i, d, f);
    }

    // ===== API for teleop =====
    public void setTargetPower(double pwr01) {
        targetPower = clamp(pwr01, 0.0, 1.0);
    }

    public void setMotorPower(double power) {
        motorShootermaster.setPower(power);
        motorShooter.setPower(power);
    }

//    public void setMotorRPM(double targetRPM) {
//        double error = targetRPM - getRPM();
//        double ff = (kV * targetRPM) + kS;
//        double fb = error * kP;
//        double power = ff + fb;
//
//        setMotorPower(power);
//    }

    public double getTicksPerSec() {
        return motorShootermaster.getVelocity();

    }

    public double getRPM() {
        return ((getTicksPerSec()/MOTOR_TICKS_PER_REV) * 60) / MOTOR_GEAR_RATIO;
    }

    public void setTargetAngle(double angle01) {
        targetAngle = clamp(angle01, SERVO_MIN, SERVO_MAX);

        // >>> ADDED: dacă setezi manual unghiul, actualizează și baza
        baseAngle = targetAngle;
    }

    // ===== Servo helpers =====
    public void setUnghi(double pos) {
        servoPos = Range.clip(pos, SERVO_MIN, SERVO_MAX);
        unghi.setPosition(servoPos);
        targetAngle = servoPos;

        // >>> ADDED
        baseAngle = targetAngle;
    }

    public double getUnghi() { return servoPos; }

    public void unghi_sus() { setUnghi(getUnghi() - SERVO_STEP); }
    public void unghi_jos() { setUnghi(getUnghi() + SERVO_STEP); }
    public void poz0() { setUnghi(SERVO_MIN); }

    // ===== init / init_loop =====
    public void init() {
        setUnghi(servoPos);
        telemetry.addLine("Init:  select BLUE/RED (DPAD) + LOCK (CROSS).");
        setTargetPower(0.0);
        on = false;

        burstActive = false;
        droppedThisBurst = false;
        shotCount = 0;
        burstHomeAngle = getUnghi();

        prevInBand = false;
        dropUntilMs = 0;
        lastDropMs = 0;

        // >>> ADDED
        baseAngle = targetAngle;
    }

    public void setVelocityRPM(double rpm) {
        double ticksPerSec = rpmToTicksPerSecond(rpm);
        motorShootermaster.setVelocity(ticksPerSec);
        motorShooter.setVelocity(ticksPerSec);
    }

    public void init_loop(boolean dl, boolean dr, boolean a) {
        if (!locked) {
            if (dr && !dpadRightPrev) selectedMode = 1;
            if (dl && !dpadLeftPrev)  selectedMode = 0;
            if (a  && !aPrev)         locked = true;

            if (selectedMode == 0) { pipeline = 0; targetID = 20; }
            else { pipeline = 4; targetID = 24; }
        }

        dpadLeftPrev  = dl;
        dpadRightPrev = dr;
        aPrev         = a;

        telemetry.addLine("=== SELECT GOAL IN INIT ===");
        telemetry.addData("Mode", selectedMode == 0 ? "BLUE" : "RED");
        telemetry.addData("Pipeline", pipeline);
        telemetry.addData("TargetID", targetID);
        telemetry.addLine(locked ? "LOCKED — apasa START" : "DPAD L/R + CROSS pentru LOCK");
    }

    // ===== Telemetry =====
    public void printTelemetry() {
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("enabled", enabled);
        telemetry.addData("on", on);

        telemetry.addData("battery(V)", "%.2f", batteryVoltage);
        telemetry.addData("distance", "%.2f", distance);

        double drop = computeVoltageDrop();
        telemetry.addData("voltDrop", "%.4f", drop);

        telemetry.addData("targetPower", "%.2f", targetPower);
        telemetry.addData("TARGET", "%.2f", TARGET);

        telemetry.addData("PID Power", "%.2f", POWER);
        telemetry.addData("In Band", Math.abs(TARGET - wheelVelocity) < DROP_THRESHOLD);

        telemetry.addData("autoAngle", autoEnabled);
        telemetry.addData("autoCircle", autoCircleAngle);

        telemetry.addData("burstActive", burstActive);
        telemetry.addData("shotCount", shotCount);

        telemetry.addData("rawVelocity (deg/s)", "%.2f", rawVelocity);
        telemetry.addData("wheelVelocity (deg/s)", "%.2f", wheelVelocity);

        telemetry.addData("dropActive", System.currentTimeMillis() < dropUntilMs);
        telemetry.addData("dropSign", DROP_SIGN);
        telemetry.addData("dropThr", "%.2f", DROP_THRESHOLD);

        // >>> ADDED (util pt debug)
        telemetry.addData("baseAngle", "%.3f", baseAngle);
        telemetry.addData("targetAngle(out)", "%.3f", targetAngle);
    }
}
