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
import util.PIDFController;

import java.util.Iterator;

@Config
public class Sortator {

    public final DcMotorEx motorSortator;
    public final Telemetry telemetry;
    public final MagneticLimitSwitch limitSwitch;
    public final SenzorCuloare senzorCuloare;

    // Encodere / pasi
    public static int TICKS_PER_REV = 538;
    public static int STEP_TICKS = 180;

    // ===== RPM telemetry (real) =====
    public double rpm = 0.0;

    // PID extern (custom) - pentru control pe poziție (MOVE)
    public static double kP = 0.016;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kF = 0.0;

    public static double MAX_POWER = 0.8;
    public static double MIN_POWER = 0.1;
    public static int DEADBAND_TICKS = 1;
    public static int MIN_POWER_ERROR_THRESHOLD = 13;
    public static double HOLD_OUTPUT_EPS = 0.03;

    // ============================
    // HOLD "silent" (după HOME)
    // ============================
    public static int HOLD_BAND_TICKS = 6;       // în bandă -> motor 0 (silent)
    public static int HOLD_WAKE_TICKS = 10;      // histerezis: începe corecția peste asta
    public static double HOLD_MAX_POWER = 0.25;  // să nu smucească în hold
    public static double HOLD_MIN_POWER = 0.08;  // să învingă frecarea când corectează
    public static double HOLD_KP = 0.010;        // mai mic decât kP de move
    public static double HOLD_KI = 0.0;
    public static double HOLD_KD = 0.0;          // recomand 0 pentru "no buzz"
    public static double HOLD_KF = 0.0;

    private boolean holdActiveCorrection = false;

    // ✅ PIDF intern (SDK) pentru velocity control (setVelocity) - CONFIG FRIENDLY
    public static double vP = 0.006;
    public static double vI = 0.00006;
    public static double vD = 0.00005;
    public static double vF = 12.225;

    // cache ca să reaplicăm doar când se schimbă în Dashboard
    private double lastvP = Double.NaN, lastvI = Double.NaN, lastvD = Double.NaN, lastvF = Double.NaN;

    // ===== Battery compensation for CONTINUOUS RPM =====
    private VoltageSensor batterySensor;
    public double batteryVoltage = 0.0;

    // at 12.4V -> NO compensation
    public static double FEED_V0 = 12.4;
    public static double FEED_RPM_PER_VOLT = 0;   // ex: 5
    public static double FEED_RPM_DROP_MAX = 0;   // ex: 10

    // debug
    public double lastContinuousRpmRequested = 0.0;
    public double lastContinuousRpmApplied = 0.0;
    public double lastContinuousRpmDrop = 0.0;

    // ===== HOMING anti-spasme (DEBOUNCE + SETTLE) =====
    public static double HOME_POWER_CCW = 0.13;
    public static double HOME_POWER_CW  = -0.13;
    public static double HOME_DEBOUNCE_S = 0.03;
    public static double HOME_SETTLE_S   = 0.06;

    private double limitPressedSince = -1.0;
    private boolean homeStopIssued = false;
    private double homeStopTime = 0.0;

    // State
    public boolean homingInCurs = false;
    public boolean rotireInCurs = false;
    public boolean homed = false;

    enum Directie { NONE, CW, CCW }
    public Directie directieCurenta = Directie.NONE;

    enum MoveType { NONE, STEP, ONE_SHOT, FULL_ROT, SHOOT_POS }
    private MoveType moveType = MoveType.NONE;

    // homing helpers
    public boolean homeAfterRotationRequest = false;
    public boolean limitHitDuringRotation = false;
    public boolean lastLimit = false;
    public boolean seenInactive = false;

    // PID custom
    public PIDFController pid;      // MOVE PID
    public PIDFController pidHold;  // HOLD PID (silent)
    public double targetPosition = 0;
    public boolean pidAngleControlEnabled = true;

    // DT real (nu constant)
    private double lastUpdateTime = -1.0;

    // timeouts
    public static double ROTATE_TIMEOUT_S = 3.5;
    public static double HOMING_TIMEOUT_S = 3.5;

    public double lastStateChangeTime = 0;

    // ===== COLOR SEEK =====
    public boolean cautareCuloareInCurs = false;
    public SenzorCuloare.CuloareArtefact culoareTintita = null;

    public static double SEEK_POWER_CW = -0.18;
    public static double SEEK_TIMEOUT_S = 3.0;

    public static int SEEK_STABLE_READS = 4;
    private int stableReads = 0;
    private double seekStartTime = 0;

    // ============================================================
    // ===== ADDED: Distance-based continuous RPM for BURST =======
    // ============================================================
    private CalculateTargetPower calcPower = null;

    public void setCalculateTargetPower(CalculateTargetPower calc) {
        this.calcPower = calc;
    }

    public static double FEED_DIST_SWITCH = 105.0;
    public static double FEED_RPM_NEAR = 210.0;
    public static double FEED_RPM_FAR  = 95;

    public static double FEED_HYST = 3.0;

    public static double FEED_DIST_ALPHA = 0.25;
    private double feedDistFilt = FEED_DIST_SWITCH;

    private boolean feedIsNear = true;

    private double lastBurstRpmSet = Double.NaN;
    public static double BURST_RPM_EPS = 1.0;

    /** Call ONLY while burst is ON (right_bumper held). */
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

    /** Reset call at end of burst so next burst can pick fresh. */
    public void endBurstFeed() {
        lastBurstRpmSet = Double.NaN;
    }
    // ============================================================

    public double nowSeconds() {
        return System.nanoTime() / 1e9;
    }

    public Sortator(HardwareMap hw, Telemetry t) {
        this.telemetry = t;

        motorSortator = hw.get(DcMotorEx.class, "motorSortator");
        limitSwitch = new MagneticLimitSwitch(hw, "limitSwitch");
        senzorCuloare = new SenzorCuloare(hw, telemetry);

        motorSortator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSortator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSortator.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSortator.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(vP, vI, vD, vF)
        );
        lastvP = vP; lastvI = vI; lastvD = vD; lastvF = vF;

        Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
        batterySensor = it.hasNext() ? it.next() : null;

        // MOVE PID
        pid = new PIDFController(kP, kI, kD, kF);

        // HOLD PID (silent)
        pidHold = new PIDFController(HOLD_KP, HOLD_KI, HOLD_KD, HOLD_KF);

        int currentTicks = motorSortator.getCurrentPosition();
        targetPosition = currentTicks;
        pid.setSetpoint(targetPosition);
        pidHold.setSetpoint(targetPosition);

        lastLimit = limitSwitch.isPressed();
        seenInactive = !lastLimit;
        homed = false;

        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Reapply velocity PIDF live (Dashboard)
    // ============================
    private void applyVelocityPIDFIfChanged() {
        if (vP != lastvP || vI != lastvI || vD != lastvD || vF != lastvF) {
            motorSortator.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(vP, vI, vD, vF)
            );
            lastvP = vP; lastvI = vI; lastvD = vD; lastvF = vF;
        }
    }

    // ============================
    // Battery helpers
    // ============================
    private double readBatteryVoltage() {
        if (batterySensor == null) return 0.0;
        double v = batterySensor.getVoltage();
        if (v < 1.0) return 0.0;
        return v;
    }

    private double compensateContinuousRpm(double rpmOut) {
        batteryVoltage = readBatteryVoltage();

        if (batteryVoltage <= 1.0) {
            lastContinuousRpmRequested = rpmOut;
            lastContinuousRpmApplied = rpmOut;
            lastContinuousRpmDrop = 0.0;
            return rpmOut;
        }

        double dv = batteryVoltage - FEED_V0;
        if (dv <= 0.0) {
            lastContinuousRpmRequested = rpmOut;
            lastContinuousRpmApplied = rpmOut;
            lastContinuousRpmDrop = 0.0;
            return rpmOut;
        }

        double drop = dv * FEED_RPM_PER_VOLT;
        drop = Range.clip(drop, 0.0, FEED_RPM_DROP_MAX);

        double corrected = rpmOut - drop;
        if (corrected < 0.0) corrected = 0.0;

        lastContinuousRpmRequested = rpmOut;
        lastContinuousRpmApplied = corrected;
        lastContinuousRpmDrop = drop;

        return corrected;
    }

    // ============================
    // Utilitare state
    // ============================
    private void cancelColorSeek() {
        cautareCuloareInCurs = false;
        culoareTintita = null;
        stableReads = 0;
    }

    private void beginAnglePIDMove(double newTarget, Directie dir, MoveType type) {
        if (homingInCurs) return;

        cancelColorSeek();

        targetPosition = newTarget;
        pid.setSetpoint(targetPosition);
        pidAngleControlEnabled = true;

        // când începe o mișcare, nu mai suntem în "hold correction"
        holdActiveCorrection = false;

        rotireInCurs = true;
        directieCurenta = dir;
        limitHitDuringRotation = false;

        moveType = type;
        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Homing (DEBOUNCE + SETTLE)
    // ============================
    public void home() { // CCW
        cancelColorSeek();

        boolean pressed = limitSwitch.isPressed();
        if (pressed) {
            resetEncoderAtHome();
            return;
        }

        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSortator.setPower(HOME_POWER_CCW); // CCW

        homingInCurs = true;
        rotireInCurs = false;
        homed = false;
        directieCurenta = Directie.CCW;
        moveType = MoveType.NONE;

        pidAngleControlEnabled = false;

        limitPressedSince = -1.0;
        homeStopIssued = false;
        homeStopTime = 0.0;

        lastLimit = pressed;
        seenInactive = !pressed;
        lastStateChangeTime = nowSeconds();
    }

    public void homeReverse() { // CW
        cancelColorSeek();

        boolean pressed = limitSwitch.isPressed();
        if (pressed) {
            resetEncoderAtHome();
            return;
        }

        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSortator.setPower(HOME_POWER_CW); // CW

        homingInCurs = true;
        rotireInCurs = false;
        homed = false;
        directieCurenta = Directie.CW;
        moveType = MoveType.NONE;

        pidAngleControlEnabled = false;

        limitPressedSince = -1.0;
        homeStopIssued = false;
        homeStopTime = 0.0;

        lastLimit = pressed;
        seenInactive = !pressed;
        lastStateChangeTime = nowSeconds();
    }

    public void resetEncoderAtHome() {
        motorSortator.setPower(0);
        motorSortator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSortator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        directieCurenta = Directie.NONE;

        // HOME = 0
        targetPosition = 0;

        // resetează PID-uri și intră în HOLD silent
        pid.reset();
        pid.setSetpoint(targetPosition);

        pidHold.reset();
        pidHold.setSetpoint(targetPosition);

        pidAngleControlEnabled = true;   // HOLD activ după home
        holdActiveCorrection = false;    // începe silent

        homed = true;
        rotireInCurs = false;
        homingInCurs = false;
        homeAfterRotationRequest = false;
        limitHitDuringRotation = false;
        moveType = MoveType.NONE;

        limitPressedSince = -1.0;
        homeStopIssued = false;
        homeStopTime = 0.0;

        cancelColorSeek();
        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Miscari cu PID pe unghi
    // ============================
    private void startRotateCW() {
        if (homingInCurs) return;
        beginAnglePIDMove(motorSortator.getCurrentPosition() - STEP_TICKS, Directie.CW, MoveType.STEP);
    }

    public void startRotateCCW() {
        if (homingInCurs) return;
        beginAnglePIDMove(motorSortator.getCurrentPosition() + STEP_TICKS, Directie.CCW, MoveType.STEP);
    }

    public void oneShot() { // feed CCW
        if (homingInCurs) return;
        beginAnglePIDMove(motorSortator.getCurrentPosition() + 120, Directie.CCW, MoveType.ONE_SHOT);
    }

    public void oRotatieSortator() { // o rotatie CCW
        if (homingInCurs) return;
        beginAnglePIDMove(motorSortator.getCurrentPosition() + TICKS_PER_REV, Directie.CCW, MoveType.FULL_ROT);
    }

    public void opozitieSortatorCW() { // “un pic CW”
        if (homingInCurs) return;
        beginAnglePIDMove(motorSortator.getCurrentPosition() - 120, Directie.CW, MoveType.FULL_ROT);
    }

    public void shootPosition() { // teleop shoot position CW
        if (homingInCurs) return;
        int x = motorSortator.getCurrentPosition();
        motorSortator.setTargetPosition(x - 95);
        motorSortator.setPower(0.5);
        motorSortator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // ============================
    // Cautare culoare
    // ============================
    public void aduCuloareLaCamera3(SenzorCuloare.CuloareArtefact culoareDorita) {
        if (culoareDorita == null) return;
        if (homingInCurs) return;

        rotireInCurs = false;
        pidAngleControlEnabled = false;
        moveType = MoveType.NONE;

        holdActiveCorrection = false;

        culoareTintita = culoareDorita;
        cautareCuloareInCurs = true;
        stableReads = 0;

        seekStartTime = nowSeconds();
        lastStateChangeTime = seekStartTime;

        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSortator.setPower(SEEK_POWER_CW);
        directieCurenta = Directie.CW;
    }

    public void opresteCautareaCuloare() {
        motorSortator.setPower(0);
        cancelColorSeek();
        directieCurenta = Directie.NONE;
        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Rotatie continua RPM (with battery compensation)
    // ============================
    public void sortatorContinuousRPM(double rpmOut) {
        cancelColorSeek();

        pidAngleControlEnabled = false;
        rotireInCurs = false;
        homingInCurs = false;
        moveType = MoveType.NONE;
        directieCurenta = Directie.NONE;

        holdActiveCorrection = false;

        motorSortator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rpmCorrected = compensateContinuousRpm(rpmOut);

        double ticksPerSec = (rpmCorrected / 60.0) * TICKS_PER_REV;
        motorSortator.setVelocity(ticksPerSec);

        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Stop
    // ============================
    public void stop() {
        motorSortator.setPower(0);
        motorSortator.setVelocity(0);
        motorSortator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotireInCurs = false;
        homingInCurs = false;
        homeAfterRotationRequest = false;
        limitHitDuringRotation = false;
        directieCurenta = Directie.NONE;

        pidAngleControlEnabled = false;
        moveType = MoveType.NONE;

        holdActiveCorrection = false;

        limitPressedSince = -1.0;
        homeStopIssued = false;
        homeStopTime = 0.0;

        cancelColorSeek();
        lastStateChangeTime = nowSeconds();
    }

    // ============================
    // Update loop
    // ============================
    public void update() {
        applyVelocityPIDFIfChanged();

        double now = nowSeconds();
        double timeInState = now - lastStateChangeTime;
        boolean currentLimit = limitSwitch.isPressed();

        // DT real (important pentru stabilitate)
        double dt = (lastUpdateTime < 0) ? 0.02 : (now - lastUpdateTime);
        lastUpdateTime = now;
        dt = Range.clip(dt, 0.005, 0.05);

        // keep batteryVoltage fresh (for telemetry)
        batteryVoltage = readBatteryVoltage();

        // current RPM telemetry (real)
        rpm = (motorSortator.getVelocity() / (double) TICKS_PER_REV) * 60.0;

        // ===== timeouts & logic =====
        if (rotireInCurs && timeInState > ROTATE_TIMEOUT_S) {
            motorSortator.setPower(0);
            rotireInCurs = false;
            homeAfterRotationRequest = false;
            limitHitDuringRotation = false;
            directieCurenta = Directie.NONE;
            moveType = MoveType.NONE;

            pidAngleControlEnabled = false;
            targetPosition = motorSortator.getCurrentPosition();
            pid.setSetpoint(targetPosition);
            pidHold.setSetpoint(targetPosition);

            holdActiveCorrection = false;

            lastStateChangeTime = nowSeconds();
        }

        if (homingInCurs && timeInState > HOMING_TIMEOUT_S) {
            motorSortator.setPower(0);
            homingInCurs = false;
            homed = false;

            pidAngleControlEnabled = false;
            moveType = MoveType.NONE;

            holdActiveCorrection = false;

            limitPressedSince = -1.0;
            homeStopIssued = false;
            homeStopTime = 0.0;

            lastStateChangeTime = nowSeconds();
        }

        // ============================
        // HOMING block (anti-spasme)
        // ============================
        if (homingInCurs) {
            if (currentLimit && !homeStopIssued) {
                motorSortator.setPower(0);
                homeStopIssued = true;
                homeStopTime = now;
            }

            if (currentLimit) {
                if (limitPressedSince < 0) limitPressedSince = now;
            } else {
                limitPressedSince = -1.0;
            }

            boolean settled = homeStopIssued && (now - homeStopTime) >= HOME_SETTLE_S;
            if (settled) {
                resetEncoderAtHome();
                telemetry.addData("Sortator V", "%.2f", batteryVoltage);
                telemetry.addData("RPM real", "%.1f", rpm);
                telemetry.addData("vP vI vD vF", "%.4f %.6f %.4f %.4f", vP, vI, vD, vF);
                telemetry.update();
                return;
            }

            lastLimit = currentLimit;
            return;
        }

        // ============================
        // COLOR SEEK
        // ============================
        if (cautareCuloareInCurs) {
            if (now - seekStartTime > SEEK_TIMEOUT_S) {
                motorSortator.setPower(0);
                cancelColorSeek();
                directieCurenta = Directie.NONE;
                lastStateChangeTime = nowSeconds();
                telemetry.update();
                return;
            }

            SenzorCuloare.CuloareArtefact citita = senzorCuloare.detecteazaCuloare();
            if (citita == culoareTintita) stableReads++;
            else stableReads = 0;

            if (stableReads >= SEEK_STABLE_READS) {
                motorSortator.setPower(0);
                motorSortator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                cancelColorSeek();
                directieCurenta = Directie.NONE;
                lastStateChangeTime = nowSeconds();
                telemetry.update();
                return;
            }

            motorSortator.setPower(SEEK_POWER_CW);
            lastLimit = currentLimit;

            telemetry.addData("Sortator V", "%.2f", batteryVoltage);
            telemetry.addData("RPM real", "%.1f", rpm);
            telemetry.addData("vP vI vD vF", "%.4f %.6f %.4f %.4f", vP, vI, vD, vF);
            telemetry.update();
            return;
        }

        if (rotireInCurs && currentLimit) limitHitDuringRotation = true;

        // ============================
        // PID / HOLD logic
        // ============================
        if (pidAngleControlEnabled) {
            int pos = motorSortator.getCurrentPosition();
            double error = targetPosition - pos;
            double absErr = Math.abs(error);

            boolean isMove = rotireInCurs || (moveType != MoveType.NONE);

            if (isMove) {
                // ===== MOVE PID (ca înainte) =====
                pid.setSetpoint(targetPosition);
                double output = pid.update(pos, dt, 0.0);

                if (Math.abs(error) <= DEADBAND_TICKS) {
                    if (Math.abs(output) < HOLD_OUTPUT_EPS) output = 0.0;
                } else {
                    if (output > MAX_POWER) output = MAX_POWER;
                    if (output < -MAX_POWER) output = -MAX_POWER;

                    if (Math.abs(error) > MIN_POWER_ERROR_THRESHOLD && Math.abs(output) < MIN_POWER) {
                        output = Math.copySign(MIN_POWER, output);
                    }
                }

                motorSortator.setPower(output);

                if (rotireInCurs && Math.abs(error) <= DEADBAND_TICKS) {
                    rotireInCurs = false;

                    directieCurenta = Directie.NONE;
                    moveType = MoveType.NONE;

                    if (homeAfterRotationRequest) {
                        homeAfterRotationRequest = false;

                        if (!limitHitDuringRotation) {
                            limitHitDuringRotation = false;
                            homeReverse();
                            lastLimit = currentLimit;

                            telemetry.update();
                            return;
                        }
                        limitHitDuringRotation = false;
                    }

                    // după ce s-a terminat move-ul, intrăm în hold silent (dacă suntem homed)
                    holdActiveCorrection = false;
                    pidHold.reset();
                    pidHold.setSetpoint(targetPosition);

                    lastStateChangeTime = nowSeconds();
                }

            } else if (homed) {
                // ===== HOLD silent după HOME =====

                // pornește corecția doar dacă a ieșit din zona "wake"
                if (!holdActiveCorrection) {
                    if (absErr <= HOLD_WAKE_TICKS) {
                        motorSortator.setPower(0.0); // silent
                    } else {
                        holdActiveCorrection = true;
                        pidHold.reset();
                        pidHold.setSetpoint(targetPosition);
                    }
                }

                if (holdActiveCorrection) {
                    // dacă a intrat în banda de silent, oprește complet și revino la silent
                    if (absErr <= HOLD_BAND_TICKS) {
                        motorSortator.setPower(0.0);
                        holdActiveCorrection = false;
                        pidHold.reset();
                        pidHold.setSetpoint(targetPosition);
                    } else {
                        pidHold.setSetpoint(targetPosition);
                        double out = pidHold.update(pos, dt, 0.0);

                        // limitări blânde (hold)
                        out = Range.clip(out, -HOLD_MAX_POWER, HOLD_MAX_POWER);

                        // min power doar când chiar corectezi (ca să nu bâzâie în bandă)
                        if (Math.abs(out) < HOLD_MIN_POWER) {
                            out = Math.copySign(HOLD_MIN_POWER, out);
                        }

                        motorSortator.setPower(out);
                    }
                }
            } else {
                // nu suntem homed -> nu încercăm hold (evită surprize)
                motorSortator.setPower(0.0);
                holdActiveCorrection = false;
            }
        }

        lastLimit = currentLimit;

        telemetry.addData("Sortator V", "%.2f", batteryVoltage);
        telemetry.addData("FeedRPM req", "%.1f", lastContinuousRpmRequested);
        telemetry.addData("FeedRPM drop", "%.1f", lastContinuousRpmDrop);
        telemetry.addData("FeedRPM applied", "%.1f", lastContinuousRpmApplied);
        telemetry.addData("RPM real", "%.1f", rpm);
        telemetry.addData("vP vI vD vF", "%.4f %.6f %.4f %.4f", vP, vI, vD, vF);
        telemetry.addData("HOLD activeCorr", holdActiveCorrection);
        telemetry.addData("Homed", homed);
        telemetry.update();
    }

    public void intakeSiMemoreaza() {
        if (rotireInCurs || homingInCurs || cautareCuloareInCurs) return;
        startRotateCW();
    }
}

