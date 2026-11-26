package hardwareParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import senzori.MagneticLimitSwitch;
import senzori.SenzorCuloare;

import java.util.Iterator;

/**
 * SortatorTBE — Through-Bore Encoder + magnetic limit switch.
 *
 * homeAbsolute() — o dată la init, cu magnet → setează 0.
 * goToIntake()   — revenire la 0 FĂRĂ magnet, cu velocity PROPORȚIONAL:
 *                  tare departe de 0, blând aproape de 0 (nu trece de țintă).
 *                  Semnul direcției vine automat din eroare.
 *                  Dacă merge INVERS, fă GOTO_KP negativ.
 */
@Config
public class SortatorTBE {

    // =========================================================================
    // HARDWARE
    // =========================================================================

    public final DcMotorEx motorSortator;
    public final MagneticLimitSwitch limitSwitch;
    public final SenzorCuloare senzorCuloare;
    public final Telemetry telemetry;

    // =========================================================================
    // CONSTANTE POZIȚIONARE
    // =========================================================================

    public static int TICKS_PER_REV = 8192;
    public static int STEP_TICKS = 2740;
    public static int SHOOT_DELTA_TICKS = -1446;
    public static int ONE_SHOT_TICKS = 1827;

    /** P pentru RUN_TO_POSITION (folosit la shoot). */
    public static double POS_P = 30.0;

    // ── REVENIRE LA INTAKE (velocity proporțional) ───────────────────────────
    /** Cât de tare împinge per tick de eroare. Dacă merge INVERS → fă-l negativ. */
    public static double GOTO_KP  = -0.0008;
    /** Putere max (departe de 0). */
    public static double GOTO_MAX = 1.0;
    /** Putere min (aproape de 0, să învingă frâna). */
    public static double GOTO_MIN = 0.18;
    /** Cât de aproape de 0 = "ajuns" (ticks). */
    public static int    GOTO_TOLERANCE = 30;

    private int intakePositionTicks = 0;

    // =========================================================================
    // RPM telemetry
    // =========================================================================

    public double rpm = 0.0;

    // =========================================================================
    // PIDF VITEZĂ (burst)
    // =========================================================================

    public static double vP = 0.006;
    public static double vI = 0.00006;
    public static double vD = 0.00005;
    public static double vF = 0.80;

    private double lastvP = Double.NaN, lastvI = Double.NaN;
    private double lastvD = Double.NaN, lastvF = Double.NaN;

    // =========================================================================
    // COMPENSARE BATERIE
    // =========================================================================

    private VoltageSensor batterySensor;
    public  double batteryVoltage = 0.0;

    public static double FEED_V0           = 12.4;
    public static double FEED_RPM_PER_VOLT = 0;
    public static double FEED_RPM_DROP_MAX = 0;

    public double lastContinuousRpmRequested = 0.0;
    public double lastContinuousRpmApplied   = 0.0;
    public double lastContinuousRpmDrop      = 0.0;

    // =========================================================================
    // STARE
    // =========================================================================

    public boolean homed        = false;
    public boolean rotireInCurs = false;
    public boolean justHomed    = false;

    enum Directie { NONE, CW, CCW }
    public Directie directieCurenta = Directie.NONE;

    private double lastUpdateTime = -1.0;

    public static double ROTATE_TIMEOUT_S = 3.5;
    public double lastStateChangeTime = 0;

    // =========================================================================
    // HOMING
    // =========================================================================

    public static double HOME_POWER_CW    = -0.13;
    public static double HOME_DEBOUNCE_S  = 0.03;
    public static double HOME_SETTLE_S    = 0.15;
    public static double HOMING_TIMEOUT_S = 10.0;

    private boolean homingInCurs      = false;
    private double  limitPressedSince = -1.0;
    private boolean homeStopIssued    = false;
    private double  homeStopTime      = 0.0;
    private boolean lastLimit         = false;
    private boolean seenInactive      = false;

    // =========================================================================
    // COLOR SEEK
    // =========================================================================

    public boolean cautareCuloareInCurs = false;
    public SenzorCuloare.CuloareArtefact culoareTintita = null;

    public static double SEEK_POWER_CW     = -0.18;
    public static double SEEK_TIMEOUT_S    = 3.0;
    public static int    SEEK_STABLE_READS = 4;

    private int    stableReads   = 0;
    private double seekStartTime = 0;

    // =========================================================================
    // BURST RPM
    // =========================================================================

    private CalculateTargetPower calcPower = null;

    public static double FEED_DIST_SWITCH = 105.0;
    public static double FEED_RPM_NEAR    = 215.0;
    public static double FEED_RPM_FAR     = 110.0;
    public static double FEED_HYST        = 3.0;
    public static double FEED_DIST_ALPHA  = 0.25;

    private double  feedDistFilt    = FEED_DIST_SWITCH;
    private boolean feedIsNear      = true;
    private double  lastBurstRpmSet = Double.NaN;
    public static double BURST_RPM_EPS = 1.0;

    public void setCalculateTargetPower(CalculateTargetPower calc) {
        this.calcPower = calc;
    }

    // =========================================================================
    // UTILITAR
    // =========================================================================

    public double nowSeconds() {
        return System.nanoTime() / 1e9;
    }

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    public SortatorTBE(HardwareMap hw, Telemetry t) {
        this.telemetry = t;

        motorSortator = hw.get(DcMotorEx.class, "motorSortator");
        limitSwitch   = new MagneticLimitSwitch(hw, "limitSwitch");
        senzorCuloare = new SenzorCuloare(hw, t);

        motorSortator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorSortator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorSortator.setDirection(DcMotorEx.Direction.FORWARD);

        motorSortator.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(vP, vI, vD, vF)
        );
        lastvP = vP; lastvI = vI; lastvD = vD; lastvF = vF;

        motorSortator.setPositionPIDFCoefficients(POS_P);

        Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        batterySensor = it.hasNext() ? it.next() : null;

        intakePositionTicks = 0;

        lastLimit    = limitSwitch.isPressed();
        seenInactive = !lastLimit;
        homed        = false;

        lastStateChangeTime = nowSeconds();
    }

    // =========================================================================
    // REAPLIERE VELOCITY PIDF
    // =========================================================================

    private void applyVelocityPIDFIfChanged() {
        if (vP != lastvP || vI != lastvI || vD != lastvD || vF != lastvF) {
            motorSortator.setPIDFCoefficients(
                    DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(vP, vI, vD, vF)
            );
            lastvP = vP; lastvI = vI; lastvD = vD; lastvF = vF;
        }
    }

    // =========================================================================
    // COMPENSARE BATERIE
    // =========================================================================

    private double readBatteryVoltage() {
        if (batterySensor == null) return 0.0;
        double v = batterySensor.getVoltage();
        return (v < 1.0) ? 0.0 : v;
    }

    private double compensateContinuousRpm(double rpmOut) {
        batteryVoltage = readBatteryVoltage();

        if (batteryVoltage <= 1.0) {
            lastContinuousRpmRequested = rpmOut;
            lastContinuousRpmApplied   = rpmOut;
            lastContinuousRpmDrop      = 0.0;
            return rpmOut;
        }

        double dv = batteryVoltage - FEED_V0;
        if (dv <= 0.0) {
            lastContinuousRpmRequested = rpmOut;
            lastContinuousRpmApplied   = rpmOut;
            lastContinuousRpmDrop      = 0.0;
            return rpmOut;
        }

        double drop = Range.clip(dv * FEED_RPM_PER_VOLT, 0.0, FEED_RPM_DROP_MAX);
        double corrected = Math.max(0.0, rpmOut - drop);

        lastContinuousRpmRequested = rpmOut;
        lastContinuousRpmApplied   = corrected;
        lastContinuousRpmDrop      = drop;
        return corrected;
    }

    // =========================================================================
    // UTILITAR INTERN
    // =========================================================================

    private void cancelColorSeek() {
        cautareCuloareInCurs = false;
        culoareTintita       = null;
        stableReads          = 0;
    }

    // =========================================================================
    // HOMING
    // =========================================================================

    public void homeAbsolute() {
        cancelColorSeek();
        rotireInCurs = false;

        boolean pressed = limitSwitch.isPressed();
        if (pressed) {
            resetEncoderAtHome();
            return;
        }

        motorSortator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorSortator.setPower(HOME_POWER_CW);

        homingInCurs    = true;
        homed           = false;
        directieCurenta = Directie.CW;

        limitPressedSince = -1.0;
        homeStopIssued    = false;
        homeStopTime      = 0.0;

        lastLimit    = pressed;
        seenInactive = !pressed;
        lastStateChangeTime = nowSeconds();
    }

    private void resetEncoderAtHome() {
        motorSortator.setPower(0);
        motorSortator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorSortator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorSortator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        directieCurenta = Directie.NONE;

        intakePositionTicks = 0;

        homed        = true;
        rotireInCurs = false;
        homingInCurs = false;
        justHomed    = true;

        limitPressedSince = -1.0;
        homeStopIssued    = false;
        homeStopTime      = 0.0;

        cancelColorSeek();
        lastStateChangeTime = nowSeconds();
    }

    public void homeReverse() { homeAbsolute(); }
    public void home()        { homeAbsolute(); }

    // =========================================================================
    // REVENIRE LA INTAKE — velocity proporțional, fără magnet
    // =========================================================================

    public void goToIntake() {
        cancelColorSeek();
        motorSortator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotireInCurs        = true;
        directieCurenta     = Directie.NONE;
        lastStateChangeTime = nowSeconds();
    }

    // =========================================================================
    // COLOR SEEK
    // =========================================================================

    public void aduCuloareLaCamera3(SenzorCuloare.CuloareArtefact culoareDorita) {
        if (culoareDorita == null) return;

        rotireInCurs         = false;
        culoareTintita       = culoareDorita;
        cautareCuloareInCurs = true;
        stableReads          = 0;
        seekStartTime        = nowSeconds();
        lastStateChangeTime  = seekStartTime;

        motorSortator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorSortator.setPower(SEEK_POWER_CW);
        directieCurenta = Directie.CW;
    }

    public void opresteCautareaCuloare() {
        motorSortator.setPower(0);
        cancelColorSeek();
        directieCurenta     = Directie.NONE;
        lastStateChangeTime = nowSeconds();
    }

    // =========================================================================
    // ROTAȚIE CONTINUĂ RPM
    // =========================================================================

    public void sortatorContinuousRPM(double rpmOut) {
        cancelColorSeek();
        rotireInCurs    = false;
        directieCurenta = Directie.NONE;

        motorSortator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double rpmCorrected = compensateContinuousRpm(rpmOut);
        double ticksPerSec  = (rpmCorrected / 60.0) * TICKS_PER_REV;
        motorSortator.setVelocity(ticksPerSec);

        lastStateChangeTime = nowSeconds();
    }

    // =========================================================================
    // BURST RPM
    // =========================================================================

    public void burstFeedRPMByDistance() {
        if (calcPower == null) return;

        double d = calcPower.getDistance();
        feedDistFilt = feedDistFilt + FEED_DIST_ALPHA * (d - feedDistFilt);

        if (feedIsNear) {
            if (feedDistFilt > FEED_DIST_SWITCH + FEED_HYST) feedIsNear = false;
        } else {
            if (feedDistFilt < FEED_DIST_SWITCH - FEED_HYST) feedIsNear = true;
        }

        double desired = feedIsNear ? FEED_RPM_NEAR : FEED_RPM_FAR;

        if (Double.isNaN(lastBurstRpmSet) || Math.abs(desired - lastBurstRpmSet) > BURST_RPM_EPS) {
            sortatorContinuousRPM(desired);
            lastBurstRpmSet = desired;
        }
    }

    public void endBurstFeed() {
        lastBurstRpmSet = Double.NaN;
    }

    // =========================================================================
    // STOP
    // =========================================================================

    public void stop() {
        motorSortator.setPower(0);
        motorSortator.setVelocity(0);
        motorSortator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotireInCurs    = false;
        directieCurenta = Directie.NONE;

        cancelColorSeek();
        lastStateChangeTime = nowSeconds();
    }

    // =========================================================================
    // INTAKE + MEMOREAZĂ
    // =========================================================================

    public void intakeSiMemoreaza() { }

    /** Shoot — RUN_TO_POSITION cu posP. */
    public void shootPosition() {
        if (!homed) return;
        rotireInCurs = false;
        int target = intakePositionTicks + SHOOT_DELTA_TICKS;
        motorSortator.setTargetPosition(target);
        motorSortator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorSortator.setPower(0.8);
    }

    public void startRotateCCW() {
        rotireInCurs = false;
        int target = motorSortator.getCurrentPosition() + STEP_TICKS;
        motorSortator.setTargetPosition(target);
        motorSortator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorSortator.setPower(0.8);
    }

    // =========================================================================
    // UPDATE LOOP
    // =========================================================================

    public void update() {
        applyVelocityPIDFIfChanged();

        double now         = nowSeconds();
        double timeInState = now - lastStateChangeTime;

        double dt = (lastUpdateTime < 0) ? 0.02 : (now - lastUpdateTime);
        lastUpdateTime = now;
        dt = Range.clip(dt, 0.005, 0.05);

        batteryVoltage = readBatteryVoltage();
        rpm = (motorSortator.getVelocity() / (double) TICKS_PER_REV) * 60.0;

        boolean currentLimit = limitSwitch.isPressed();

        // ─── Homing ─────────────────────────────────────────────────────────
        if (homingInCurs) {
            if (motorSortator.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                motorSortator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (timeInState > HOMING_TIMEOUT_S) {
                motorSortator.setPower(0);
                homingInCurs = false;
                homed = false;
                limitPressedSince = -1.0;
                homeStopIssued = false;
                homeStopTime = 0.0;
                lastStateChangeTime = nowSeconds();
            } else if (currentLimit && !homeStopIssued) {
                motorSortator.setPower(0);
                homeStopIssued = true;
                homeStopTime = now;
            } else if (!homeStopIssued) {
                motorSortator.setPower(HOME_POWER_CW);
            }

            if (currentLimit) {
                if (limitPressedSince < 0) limitPressedSince = now;
            } else {
                limitPressedSince = -1.0;
            }

            boolean settled = homeStopIssued && (now - homeStopTime) >= HOME_SETTLE_S;
            if (settled) {
                resetEncoderAtHome();
                lastLimit = currentLimit;
                printTelemetry();
                return;
            }

            lastLimit = currentLimit;
            printTelemetry();
            return;
        }

        // ─── Color seek ───────────────────────────────────────────────────
        if (cautareCuloareInCurs) {
            if (now - seekStartTime > SEEK_TIMEOUT_S) {
                motorSortator.setPower(0);
                cancelColorSeek();
                directieCurenta     = Directie.NONE;
                lastStateChangeTime = nowSeconds();
                return;
            }

            SenzorCuloare.CuloareArtefact citita = senzorCuloare.detecteazaCuloare();
            if (citita == culoareTintita) stableReads++;
            else stableReads = 0;

            if (stableReads >= SEEK_STABLE_READS) {
                motorSortator.setPower(0);
                motorSortator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                cancelColorSeek();
                directieCurenta     = Directie.NONE;
                lastStateChangeTime = nowSeconds();
                return;
            }

            motorSortator.setPower(SEEK_POWER_CW);
            printTelemetry();
            return;
        }

        // ─── REVENIRE LA INTAKE (0): velocity proporțional ─────────────────
        if (rotireInCurs) {
            int err = motorSortator.getCurrentPosition() - intakePositionTicks;

            if (Math.abs(err) <= GOTO_TOLERANCE) {
                motorSortator.setPower(0);
                rotireInCurs    = false;
                directieCurenta = Directie.NONE;
                lastStateChangeTime = now;
            } else {
                // putere proporțională cu eroarea; semnul vine din err * GOTO_KP
                double p = err * GOTO_KP;
                p = Range.clip(p, -GOTO_MAX, GOTO_MAX);
                // putere minimă să învingă frâna aproape de 0
                if (Math.abs(p) < GOTO_MIN) p = Math.signum(p) * GOTO_MIN;
                motorSortator.setPower(p);
            }
            printTelemetry();
            return;
        }

        printTelemetry();
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    private void printTelemetry() {
        telemetry.addData("[SortatorTBE] Homed",        homed);
        telemetry.addData("[SortatorTBE] motorPos",     motorSortator.getCurrentPosition());
        telemetry.addData("[SortatorTBE] rotireInCurs", rotireInCurs);
        telemetry.addData("[SortatorTBE] RPM real",     "%.1f", rpm);
        telemetry.addData("[SortatorTBE] FeedRPM",      "%.1f", lastContinuousRpmApplied);
        telemetry.addData("[SortatorTBE] V baterie",    "%.2f", batteryVoltage);
    }

    public int getIntakePositionTicks() { return intakePositionTicks; }
}