package hardwareParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turela cu motor (6000 rpm) + encoder absolut.
 *
 * Unghiul turelei fata de robot (0 = drept in fata, pozitiv = trigonometric / spre stanga)
 * este citit de pe encoderul absolut. Odometria da unghiul absolut spre target; din el si din
 * heading-ul robotului rezulta unghiul dorit al turelei fata de robot. Un PD + feedforward pe
 * viteza de rotatie a robotului muta motorul pana cand encoderul confirma pozitia, deci erorile
 * (joc, alunecare, rotirea sasiului) sunt corectate continuu din encoder.
 *
 * API-ul public ramane compatibil cu varianta pe servo.
 */
@Config
public class turelaodometrieNew {

    // ===== Provider optional (Pose: x,y,heading) =====
    public interface PoseProvider {
        boolean isReady();
        double getHeadingRad();
        double getX();
        double getY();
    }

    private PoseProvider poseProvider = null;

    public void setPoseProvider(PoseProvider provider) {
        this.poseProvider = provider;
    }

    // ===== CALCULATE TARGET POWER =====
    private CalculateTargetPower targetPower = null;

    public void setCalculateTargetPower(CalculateTargetPower calc) {
        this.targetPower = calc;
    }

    // ===== Telemetry (optional) =====
    private Telemetry telemetry = null;

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private void telLine(String s) {
        if (telemetry != null) telemetry.addLine(s);
    }

    private void telData(String k, String fmt, Object... args) {
        if (telemetry != null) telemetry.addData(k, fmt, args);
    }

    private void telData(String k, Object v) {
        if (telemetry != null) telemetry.addData(k, v);
    }

    // ===== last valid (guard) =====
    private double lastHeadingDeg = 0.0;
    private double lastX = 0.0;
    private double lastY = 0.0;

    private double readHeadingDegRaw() {
        if (poseProvider != null && poseProvider.isReady()) {
            lastHeadingDeg = Math.toDegrees(poseProvider.getHeadingRad());
            return lastHeadingDeg;
        }
        return lastHeadingDeg;
    }

    private double readX() {
        if (poseProvider != null && poseProvider.isReady()) {
            lastX = poseProvider.getX();
            return lastX;
        }
        return lastX;
    }

    private double readY() {
        if (poseProvider != null && poseProvider.isReady()) {
            lastY = poseProvider.getY();
            return lastY;
        }
        return lastY;
    }

    // ===== Math helpers =====
    public static double wrap360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    public static double wrap180(double deg) {
        deg = wrap360(deg);
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    public static double headingToTargetDeg(
            double currentx, double currenty,
            double targetx, double targety) {

        double dx = targetx - currentx;
        double dy = targety - currenty;
        return wrap360(Math.toDegrees(Math.atan2(dy, dx)));
    }

    // ================= TUNING (Dashboard) =================
    /** Unghiul encoderului (0..360) cand turela e drept in fata. CALIBREAZA cu testAbsoluteEncoder. */
    public static double ENCODER_ZERO_DEG = 180.0;
    /** +1 / -1: sensul in care creste unghiul encoderului fata de sensul pozitiv al turelei (trigonometric). */
    public static double ENCODER_DIR = 1.0;
    /** Daca motorul fuge de tinta in loc sa mearga spre ea, pune -1. */
    public static double MOTOR_DIR = 1.0;

    /** Limitele mecanice (grade fata de robot) ca sa nu se rasuceasca cablurile. */
    public static double MIN_REL_DEG = -160.0;
    public static double MAX_REL_DEG = 160.0;

    public static double kP = 0.02;
    public static double kD = 0.0008;
    /** Feedforward pe viteza de rotatie a robotului (putere per grad/secunda). */
    public static double kFF = 0.0015;
    /** Putere minima pentru a invinge frecarea cand e in afara zonei moarte. */
    public static double kS = 0.04;
    public static double MAX_POWER = 0.8;

    /** Sub eroarea asta (grade) motorul sta; porneste din nou doar peste RESTART_ERROR_DEG. */
    public static double TOLERANCE_DEG = 1.0;
    public static double RESTART_ERROR_DEG = 2.0;

    /** Filtru pe encoder (0..1, mai mic = mai neted dar mai lent). */
    public static double ENCODER_FILTER = 0.5;
    /** Filtru pe viteza robotului. */
    public static double RATE_FILTER = 0.3;

    /** Trim manual: grade per apasare. */
    public static double TRIM_STEP_DEG = 1.0;
    public static double TRIM_MAX_DEG = 20.0;

    /** Pozitia (grade fata de robot) pentru modul EMERGENCY FIX. */
    public static double FIXA_REL_DEG = 0.0;

    // ================= HARDWARE =================
    public final DcMotor motorTurela;
    public final absoluteEncoder encoder;

    // ================= CONTROL =================
    public boolean turelaStart = false;

    // ================= TARGET =================
    public double targetx = 0.0;
    public double targety = 0.0;

    public void setTarget(double x, double y) {
        targetx = x;
        targety = y;
    }

    // ================= STATE =================
    public double currentx = 0.0;
    public double currenty = 0.0;

    public double headingRawDeg = 0.0;
    public double headingCopyDeg = 0.0;
    public double headingWrappedDeg = 0.0;

    public double targetHeadingDeg = 0.0;
    /** Eroarea de directie fata de target (field), pentru telemetrie / compatibilitate. */
    public double errorDeg = 0.0;

    public double startangle = 0.0;

    /** Unghiul dorit al turelei fata de robot. */
    public double desiredRelDeg = 0.0;
    /** Unghiul real al turelei fata de robot (din encoder, filtrat). */
    public double currentRelDeg = 0.0;
    /** Eroarea de pozitie a turelei (desired - current). */
    public double turretErrorDeg = 0.0;
    public double power = 0.0;

    private double lastTargetHeadingDeg = 0.0;
    private double filteredEnc = Double.NaN;
    private double lastTurretError = 0.0;
    private double lastHeadingForRate = Double.NaN;
    private double headingRateDegPerSec = 0.0;
    private long lastNanos = 0;
    private boolean holding = false;

    // ===== MANUAL TRIM (peste odometrie, grade) =====
    private double manualTrim = 0.0;

    public void trimLeft() {
        manualTrim = Range.clip(manualTrim + TRIM_STEP_DEG, -TRIM_MAX_DEG, TRIM_MAX_DEG);
    }

    public void trimRight() {
        manualTrim = Range.clip(manualTrim - TRIM_STEP_DEG, -TRIM_MAX_DEG, TRIM_MAX_DEG);
    }

    public void resetTrim() {
        manualTrim = 0.0;
    }

    public double getManualTrim() {
        return manualTrim;
    }

    // ===== EMERGENCY / FIX MODE =====
    private boolean turelaFixaActive = false;

    public boolean isTurelaFixaActive() {
        return turelaFixaActive;
    }

    /** Blocheaza turela la FIXA_REL_DEG fara sa strice update-ul de pose/power */
    public void enableTurelaFixa() {
        turelaFixaActive = true;
        turelaStart = false;
        manualTrim = 0.0;
    }

    /** Compatibilitate cu codul vechi */
    public void turelaFixa() {
        enableTurelaFixa();
    }

    /** Iesi din emergency/fix */
    public void disableTurelaFixa() {
        turelaFixaActive = false;
    }

    public turelaodometrieNew(HardwareMap hw) {
        motorTurela = hw.get(DcMotor.class, "motorTurela");
        motorTurela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTurela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTurela.setDirection(DcMotorSimple.Direction.FORWARD);
        encoder = new absoluteEncoder(hw);
    }

    public void init() {
        manualTrim = 0.0;
        turelaFixaActive = false;
        turelaStart = false;
        holding = false;
        filteredEnc = Double.NaN;
        lastHeadingForRate = Double.NaN;
        lastNanos = 0;
        motorTurela.setPower(0);
    }

    public void enableSearchOdometrie() {
        turelaFixaActive = false;
        turelaStart = true;
    }

    /** Opreste urmarirea si readuce turela drept in fata (controlul ramane activ in loop()). */
    public void disableSearchOdometrie() {
        turelaStart = false;
        manualTrim = 0.0;
    }

    // ================= ENCODER =================
    private double readTurretRelDeg() {
        encoder.update();
        double raw = wrap360(encoder.angle);
        if (Double.isNaN(filteredEnc)) filteredEnc = raw;
        filteredEnc = wrap360(filteredEnc + ENCODER_FILTER * wrap180(raw - filteredEnc));
        return wrap180(ENCODER_DIR * (filteredEnc - ENCODER_ZERO_DEG));
    }

    // ================= LOOP =================
    public void loop() {

        // ===== dt =====
        long now = System.nanoTime();
        double dt = (lastNanos == 0) ? 0.02 : (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt < 1e-3) dt = 1e-3;
        if (dt > 0.2) dt = 0.2;

        // ===== Citeste mereu pose-ul robotului =====
        currentx = readX();
        currenty = readY();

        headingRawDeg = readHeadingDegRaw();
        headingCopyDeg = headingRawDeg + startangle;
        headingWrappedDeg = wrap360(headingCopyDeg);

        // viteza de rotatie a robotului (filtrata) pentru feedforward
        if (!Double.isNaN(lastHeadingForRate)) {
            double rate = wrap180(headingWrappedDeg - lastHeadingForRate) / dt;
            headingRateDegPerSec += RATE_FILTER * (rate - headingRateDegPerSec);
        }
        lastHeadingForRate = headingWrappedDeg;

        // ===== Update live pentru CalculateTargetPower MEREU =====
        if (targetPower != null) {
            targetPower.updateFromTurela(currentx, currenty, headingWrappedDeg);
            targetPower.updateTarget(targetx, targety);
        }

        // ===== Unghi dorit al turelei fata de robot =====
        double dx = targetx - currentx;
        double dy = targety - currenty;
        if (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6) {
            lastTargetHeadingDeg = headingToTargetDeg(currentx, currenty, targetx, targety);
        }
        targetHeadingDeg = lastTargetHeadingDeg;
        errorDeg = wrap180(targetHeadingDeg - headingWrappedDeg);

        boolean tracking = turelaStart && !turelaFixaActive;
        double ff = 0.0;

        if (turelaFixaActive) {
            desiredRelDeg = FIXA_REL_DEG;
        } else if (tracking) {
            desiredRelDeg = errorDeg + manualTrim;
            ff = -headingRateDegPerSec * kFF;  // robotul se roteste => turela compenseaza
        } else {
            desiredRelDeg = 0.0;               // oprit: turela drept in fata
        }
        desiredRelDeg = Range.clip(desiredRelDeg, MIN_REL_DEG, MAX_REL_DEG);

        // ===== Controller pe encoderul absolut =====
        currentRelDeg = readTurretRelDeg();
        turretErrorDeg = desiredRelDeg - currentRelDeg;

        double absErr = Math.abs(turretErrorDeg);
        if (absErr <= TOLERANCE_DEG) holding = true;
        else if (absErr > RESTART_ERROR_DEG) holding = false;

        double deriv = (turretErrorDeg - lastTurretError) / dt;
        lastTurretError = turretErrorDeg;

        double p = 0.0;
        if (!holding) {
            p = kP * turretErrorDeg + kD * deriv + ff + kS * Math.signum(turretErrorDeg);
        } else if (tracking) {
            p = ff; // mentine feedforward-ul cat timp robotul se roteste
        }
        p = Range.clip(p, -MAX_POWER, MAX_POWER) * MOTOR_DIR;

        // protectie la limite: nu impinge mai departe in afara intervalului
        boolean beyondMax = currentRelDeg >= MAX_REL_DEG && (p * MOTOR_DIR) > 0;
        boolean beyondMin = currentRelDeg <= MIN_REL_DEG && (p * MOTOR_DIR) < 0;
        if (beyondMax || beyondMin) p = 0.0;

        power = p;
        motorTurela.setPower(power);

        // ===== Telemetrie =====
        telLine(turelaFixaActive ? "=== TURELA EMERGENCY FIX ===" : "=== TURELA ODOMETRIE + ENCODER ===");
        telData("Current X", "%.2f", currentx);
        telData("Current Y", "%.2f", currenty);
        telData("Heading WRAPPED (deg)", "%.2f", headingWrappedDeg);
        telData("Target X", "%.2f", targetx);
        telData("Target Y", "%.2f", targety);
        telData("Target Heading (deg)", "%.2f", targetHeadingDeg);
        telData("Error field (deg)", "%.2f", errorDeg);
        telData("Encoder (deg)", "%.1f", filteredEnc);
        telData("Turela desired rel", "%.2f", desiredRelDeg);
        telData("Turela current rel", "%.2f", currentRelDeg);
        telData("Turela error", "%.2f", turretErrorDeg);
        telData("Robot rate (deg/s)", "%.1f", headingRateDegPerSec);
        telData("ManualTrim (deg)", "%.2f", manualTrim);
        telData("Power", "%.3f", power);
        telData("Holding", holding);

        if (targetPower != null) {
            telData("Dist (in)", "%.2f", targetPower.getDistance());
            telData("Shooter Power RAW", "%.3f", targetPower.getPowerRaw());
            telData("Shooter Power FILT", "%.3f", targetPower.getPower());
        }
    }
}
