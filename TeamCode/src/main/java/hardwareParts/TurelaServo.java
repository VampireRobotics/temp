package hardwareParts;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class TurelaServo {

    // ================= HARDWARE ==================
    public final Servo servoTurela;
    public final Limelight3A limelight;
    public final Telemetry telemetry;

    // ================= RANGE MECANIC (VIRTUAL) =================
    public static final double MIN_DEG = -55.0;
    public static final double MAX_DEG = 35.0;

    // ================= SERVO MAPPING =================
    public static double SERVO_HOME_POS = 0.50;
    public static double SERVO_MIN_POS  = 0.20;
    public static double SERVO_MAX_POS  = 0.80;

    // grade/sec la power = 1
    public static double SERVO_DEG_PER_SEC_AT_FULL = 260.0;

    // ================= SEARCH =================
    public static double SEARCH_POWER = 0.10;
    public static double SEARCH_KICK_POWER = 0.20;
    public static int    SEARCH_KICK_MS = 150;
    public static double SEARCH_FLIP_PERIOD = 0.50; // sec

    private int searchDir = 1;
    private long searchKickUntilMs = 0;
    private final ElapsedTime searchTimer = new ElapsedTime();

    // ================= AIM =================
    public static double AIM_KP = 0.020;
    public static double AIM_MAX_POWER = 0.22;
    public static double AIM_MIN_POWER = 0.10;
    public static double AIM_DEADBAND_DEG = 5.0;
    public static double LOCK_TOLERANCE_DEG = 5.0;

    // ================= INIT SELECT =================
    public int selectedMode = 0;   // 0 = BLUE, 1 = RED
    public boolean locked = false;

    public int pipeline = 0;
    public int targetID = 20;

    private boolean dpadLeftPrev = false, dpadRightPrev = false, aPrev = false;
    private boolean crossPrev = false;

    // ================= STATE =================
    public boolean aimActive = false;
    public boolean searchEnabled = false;
    private boolean targetLocked = false;

    // ================= VIRTUAL ENCODER =================
    private double curDeg = 0.0;
    private double curServoPos = SERVO_HOME_POS;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ================= CONSTRUCTOR =================
    public TurelaServo(HardwareMap hw, Telemetry t) {
        telemetry = t;
        servoTurela = hw.get(Servo.class, "servoTurela");
        limelight = hw.get(Limelight3A.class, "limelight");
    }

    // ================= INIT =================
    public void init() {
        // du-te instant la mijloc
        servoTurela.setPosition(SERVO_HOME_POS);

        // sincronizează starea internă cu poziția reală
        curServoPos = SERVO_HOME_POS;
        curDeg = 0.0;

        // oprește acțiuni
        aimActive = false;
        searchEnabled = false;
        targetLocked = false;

        searchDir = 1;
        searchTimer.reset();
        searchKickUntilMs = 0;

        loopTimer.reset();
    }

    public void start() {
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        searchDir = 1;
        searchTimer.reset();
        searchKickUntilMs = 0;

        loopTimer.reset();

        targetLocked = false;
    }

    public void stop() {
        aimActive = false;
        searchEnabled = false;
        targetLocked = false;
        applyPower(0);
    }

    // ================= INIT LOOP (SELECT BLUE/RED + LOCK) =================
    public void init_loop(boolean dl, boolean dr, boolean a) {

        if (!locked) {
            if (dr && !dpadRightPrev) selectedMode = 1;
            if (dl && !dpadLeftPrev) selectedMode = 0;
            if (a && !aPrev) locked = true;

            dpadLeftPrev = dl;
            dpadRightPrev = dr;
            aPrev = a;

            if (selectedMode == 0) {
                pipeline = 0;
                targetID = 20;
            } else {
                pipeline = 4;
                targetID = 24;
            }

            limelight.pipelineSwitch(pipeline);
        }

        telemetry.addLine("=== SELECT GOAL IN INIT (SERVO) ===");
        telemetry.addData("Mode", selectedMode == 0 ? "BLUE" : "RED");
        telemetry.addData("Pipeline", pipeline);
        telemetry.addData("TargetID", targetID);
        telemetry.addData("deg", "%.1f", curDeg);
        telemetry.addData("min/max deg", "%.1f / %.1f", MIN_DEG, MAX_DEG);
        telemetry.addLine(locked ? "LOCKED — apasa START" : "DPAD L/R + A pentru LOCK");
    }

    // ================= PRESETS =================
    public void initBlue() {
        selectedMode = 0;
        pipeline = 0;
        targetID = 20;
        locked = true;

        limelight.pipelineSwitch(pipeline);

        aimActive = false;
        searchEnabled = false;
        targetLocked = false;

        searchDir = 1;
        searchTimer.reset();
        searchKickUntilMs = 0;

        loopTimer.reset();
    }

    public void initRed() {
        selectedMode = 1;
        pipeline = 4;
        targetID = 24;
        locked = true;

        limelight.pipelineSwitch(pipeline);

        aimActive = false;
        searchEnabled = false;
        targetLocked = false;

        searchDir = 1;
        searchTimer.reset();
        searchKickUntilMs = 0;

        loopTimer.reset();
    }

    // ================= LOOP =================
    public void loop() {

        // dacă nu vrei nimic, stai pe loc
        if (!aimActive && !searchEnabled) {
            targetLocked = false;
            applyPower(0);
            return;
        }

        // ===== detectie AprilTag strict pe ID selectat =====
        boolean hasTarget = false;
        double txDeg = 0.0;

        LLResult res = limelight.getLatestResult();
        if (res != null && res.isValid()) {
            List<LLResultTypes.FiducialResult> fid = res.getFiducialResults();
            if (fid != null) {
                for (LLResultTypes.FiducialResult f : fid) {
                    if (f.getFiducialId() == targetID) {
                        hasTarget = true;
                        txDeg = f.getTargetXDegrees();
                        break;
                    }
                }
            }
        }

        double power = 0.0;
        long now = System.currentTimeMillis();

        if (hasTarget) {

            // dacă aim e activ -> P control pe tx
            if (aimActive) {
                double err = txDeg;

                if (Math.abs(err) <= AIM_DEADBAND_DEG) {
                    power = 0.0;
                } else {
                    power = -AIM_KP * err;

                    if (power > AIM_MAX_POWER) power = AIM_MAX_POWER;
                    if (power < -AIM_MAX_POWER) power = -AIM_MAX_POWER;

                    if (Math.abs(power) < AIM_MIN_POWER) {
                        power = Math.copySign(AIM_MIN_POWER, power);
                    }
                }

                // limite mecanice
                if (curDeg >= MAX_DEG && power > 0) power = 0;
                if (curDeg <= MIN_DEG && power < 0) power = 0;
            } else {
                // dacă doar cauți (searchEnabled) dar nu vrei aim, stai când îl vezi
                power = 0.0;
            }

            targetLocked = (Math.abs(txDeg) < LOCK_TOLERANCE_DEG);

            // când vede target: oprește reset-ul de search
            searchTimer.reset();
            searchKickUntilMs = 0;

        } else if (searchEnabled) {

            targetLocked = false;

            boolean changedDir = false;

            // flip periodic
            if (searchTimer.seconds() >= SEARCH_FLIP_PERIOD) {
                searchTimer.reset();
                searchDir *= -1;
                changedDir = true;
            }

            // bounce între min/max
            if (curDeg >= MAX_DEG) { searchDir = -1; changedDir = true; }
            if (curDeg <= MIN_DEG) { searchDir =  1; changedDir = true; }

            if (changedDir) {
                searchKickUntilMs = now + SEARCH_KICK_MS; // kick doar la schimbare
            }

            power = searchDir * SEARCH_POWER;

            // kickstart
            if (now < searchKickUntilMs) {
                power = searchDir * SEARCH_KICK_POWER;
            }

            // hard stop
            if (curDeg >= MAX_DEG && power > 0) power = 0;
            if (curDeg <= MIN_DEG && power < 0) power = 0;

        } else {
            // aimActive true, dar search off și nu vede target -> stai
            targetLocked = false;
            power = 0.0;
        }

        applyPower(power);
    }

    // ================= POWER -> SERVO =================
    private void applyPower(double power) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (dt < 0) dt = 0;
        if (dt > 0.2) dt = 0.2;

        power = clamp(power, -1, 1);

        curDeg += power * SERVO_DEG_PER_SEC_AT_FULL * dt;
        curDeg = clamp(curDeg, MIN_DEG, MAX_DEG);

        curServoPos = SERVO_MIN_POS +
                (curDeg - MIN_DEG) / (MAX_DEG - MIN_DEG) *
                        (SERVO_MAX_POS - SERVO_MIN_POS);

        curServoPos = clamp(curServoPos, SERVO_MIN_POS, SERVO_MAX_POS);
        servoTurela.setPosition(curServoPos);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ================= HELPERS =================
    public boolean isTargetLocked() {
        return targetLocked;
    }

    public void enableSearch() {
        searchEnabled = true;
        searchTimer.reset();
        searchKickUntilMs = 0;
        loopTimer.reset();
    }

    public void disableSearch() {
        searchEnabled = false;
        searchKickUntilMs = 0;
    }

    // Toggle AIM (și pornește search automat când e ON)
    public void updateCrossToggle(boolean crossNow) {
        if (crossNow && !crossPrev) {
            aimActive = !aimActive;

            if (aimActive) {
                enableSearch(); // caută dacă nu vede tag
                targetLocked = false;
            } else {
                disableSearch();
                targetLocked = false;
                applyPower(0);
            }
        }
        crossPrev = crossNow;
    }

    // Poziții fixe (manual/auto)
    public void unghiAutoRed()  { servoTurela.setPosition(0.35); }
    public void unghiAutoBlue() { servoTurela.setPosition(0.675); }
    public void unghiHome()     { servoTurela.setPosition(0.50); }

    public void unghiMax() {servoTurela.setPosition(1.0);}
    public void unghiMin() {servoTurela.setPosition(0.0);}
}