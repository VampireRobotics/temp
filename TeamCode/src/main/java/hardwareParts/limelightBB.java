package hardwareParts;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@com.acmerobotics.dashboard.config.Config
public class limelightBB implements PolenSource {

    public static final int PIPELINE_RED_TAG = 0;
    public static final int PIPELINE_BLUE_TAG = 1;
    public static final int PIPELINE_BILE = 2;

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private int pipelineCurent = -1;
    private long lastSwitchMs = 0;
    private static final long PIPELINE_RESEND_MS = 300;        // cooldown dupa o comanda reusita
    private static final long PIPELINE_RESEND_FAIL_MS = 1500;  // cooldown cand comanda esueaza (nu inundam apelurile de retea)

    // SDK-ul face POST /pipeline-switch cu connect-timeout de 100 ms si intoarce false la orice exceptie sau cod != 200.
    // Facem noi POST-ul, pe thread separat (nu blocheaza bucla), cu timeout mai mare, si pastram motivul esecului.
    private static final String LL_URL_FALLBACK = "http://172.29.0.1:5807";
    private String llUrl = LL_URL_FALLBACK;   // se inlocuieste cu baseUrl-ul real din SDK (reflection)
    private final ExecutorService io = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "LL-pipeline-switch");
        t.setDaemon(true);
        return t;
    });
    private volatile boolean switchInFlight = false;
    private volatile int diagHttpCode = -1;
    private volatile String diagHttpErr = "-";
    private volatile boolean diagSdkOk = false;

    // ---- diagnostic (doar pentru telemetrie) ----
    private boolean diagNull = true, diagValid = false;
    private int diagPipelineReal = -1;
    private int diagNrDetectii = 0;
    private volatile boolean diagSwitchOk = false;
    private volatile int diagSwitchCount = 0;

    // ---- ultimul AprilTag vazut ----
    private boolean tagVizibil = false;
    private int tagId = -1;
    private double tagTx = 0, tagTy = 0, tagTa = 0;

    // ---- scor de alegere + lock (optional, vezi setScorAlegere) ----
    public static double SCOR_TX_DEG = 30;       // cat penalizeaza abaterea laterala (mai mic = prefera mai mult centrul)
    public static long LOCK_MS = 500;            // cat tinem lock-ul fara sa mai vedem tinta
    public static double LOCK_RADIUS_DEG = 8;    // raza (grade) in care cautam tinta blocata
    private boolean scorAlegere = false;
    private long lockSeenMs = 0;
    private double lockTx = 0, lockTy = 0;
    private String diagScoruri = "-";

    // ---- cea mai buna bila galbena ----
    private boolean bilaVizibila = false;
    private double bilaTx = 0, bilaTy = 0, bilaTa = 0;

    public limelightBB(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hw.get(Limelight3A.class, "limelight");
        llUrl = citesteBaseUrl();
        limelight.setPollRateHz(200);
        limelight.start();
        setPipeline(PIPELINE_RED_TAG);
    }

    public void setPipeline(int pipeline) {
        if (pipeline == pipelineCurent) return;
        pipelineCurent = pipeline;
        lastSwitchMs = System.currentTimeMillis();
        asyncSwitch(pipeline);
    }

    /** SDK-ul isi construieste adresa din IP-ul device-ului (nu e fix 172.29.0.1): o luam de acolo. */
    private String citesteBaseUrl() {
        try {
            java.lang.reflect.Field f = Limelight3A.class.getDeclaredField("baseUrl");
            f.setAccessible(true);
            Object v = f.get(limelight);
            if (v instanceof String && !((String) v).isEmpty()) return (String) v;
        } catch (Exception ignored) {
            // ramanem pe fallback
        }
        return LL_URL_FALLBACK;
    }

    /** Trimite comanda de schimbare pipeline pe thread separat. Ignora daca una e deja in curs. */
    private void asyncSwitch(final int idx) {
        if (switchInFlight) return;
        switchInFlight = true;
        diagSwitchCount++;
        io.execute(() -> {
            HttpURLConnection c = null;
            try {
                c = (HttpURLConnection) new URL(llUrl + "/pipeline-switch?index=" + idx).openConnection();
                c.setRequestMethod("POST");
                c.setConnectTimeout(1000);
                c.setReadTimeout(1500);
                int code = c.getResponseCode();
                diagHttpCode = code;
                diagHttpErr = "-";
                diagSwitchOk = (code == 200);
            } catch (Exception e) {
                diagHttpCode = -1;
                diagHttpErr = e.getClass().getSimpleName() + ": " + e.getMessage();
                diagSwitchOk = false;
            } finally {
                if (c != null) c.disconnect();
            }
            if (!diagSwitchOk) {
                // plan B: comanda din SDK (tot pe thread-ul asta)
                try { diagSdkOk = limelight.pipelineSwitch(idx); } catch (Exception ignored) { diagSdkOk = false; }
                if (diagSdkOk) diagSwitchOk = true;
            }
            switchInFlight = false;
        });
    }

    public void modRedTag()  { setPipeline(PIPELINE_RED_TAG); }
    public void modBlueTag() { setPipeline(PIPELINE_BLUE_TAG); }
    public void modBile()    { setPipeline(PIPELINE_BILE); }

    /** Apeleaza in fiecare loop. Citeste rezultatul in functie de pipeline-ul activ. */
    public void update() {
        tagVizibil = false;
        bilaVizibila = false;

        LLResult res = limelight.getLatestResult();
        diagNull = res == null;
        diagValid = res != null && res.isValid();
        diagPipelineReal = res != null ? res.getPipelineIndex() : -1;

        // Limelight-ul poate sta pe alt pipeline decat credem (comanda pierduta sau mutat de alt cod):
        // daca real != dorit, retrimitem comanda (cu cooldown, ca sa nu inundam Limelight-ul)
        long now = System.currentTimeMillis();
        if (res != null && pipelineCurent >= 0 && diagPipelineReal != pipelineCurent
                && !switchInFlight
                && now - lastSwitchMs > (diagSwitchOk ? PIPELINE_RESEND_MS : PIPELINE_RESEND_FAIL_MS)) {
            lastSwitchMs = now;
            asyncSwitch(pipelineCurent);
        }
        if (res != null && diagPipelineReal != pipelineCurent) return;   // rezultat de pe alt pipeline: il ignoram

        if (res == null || !res.isValid()) return;

        if (pipelineCurent == PIPELINE_BILE) {
            citesteBile(res);
        } else {
            citesteTaguri(res);
        }
    }

    private void citesteTaguri(LLResult res) {
        List<LLResultTypes.FiducialResult> fid = res.getFiducialResults();
        if (fid == null || fid.isEmpty()) return;

        // il luam pe cel mai mare (cel mai aproape)
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult f : fid) {
            if (best == null || f.getTargetArea() > best.getTargetArea()) best = f;
        }
        tagVizibil = true;
        tagId = best.getFiducialId();
        tagTx = best.getTargetXDegrees();
        tagTy = best.getTargetYDegrees();
        tagTa = best.getTargetArea();
    }

    /** Scor de alegere: bila mare (aproape) si mai spre centru castiga. */
    public static double scor(double tx, double ta) {
        return ta / (1.0 + Math.abs(tx) / SCOR_TX_DEG);
    }

    /** Activeaza alegerea dupa scor + lock pe tinta (altfel: cea mai mare arie, ca inainte). */
    public void setScorAlegere(boolean on) {
        scorAlegere = on;
        lockSeenMs = 0;
    }

    private void citesteBile(LLResult res) {
        // pipeline de tip Detector (retea neuronala)
        List<LLResultTypes.DetectorResult> det = res.getDetectorResults();
        diagNrDetectii = det == null ? 0 : det.size();
        if (det != null && !det.isEmpty()) {
            LLResultTypes.DetectorResult best = null;
            if (scorAlegere) {
                long now = System.currentTimeMillis();
                StringBuilder sb = new StringBuilder();
                for (LLResultTypes.DetectorResult d : det) {
                    sb.append(String.format("[tx%.0f ta%.1f s%.2f] ", d.getTargetXDegrees(), d.getTargetArea(),
                            scor(d.getTargetXDegrees(), d.getTargetArea())));
                }
                diagScoruri = sb.toString();

                // lock: ramanem pe aceeasi bila cat timp o vedem aproape de unde era
                if (lockSeenMs != 0 && now - lockSeenMs <= LOCK_MS) {
                    double bestDist = LOCK_RADIUS_DEG;
                    for (LLResultTypes.DetectorResult d : det) {
                        double dist = Math.hypot(d.getTargetXDegrees() - lockTx, d.getTargetYDegrees() - lockTy);
                        if (dist <= bestDist) { bestDist = dist; best = d; }
                    }
                }
                // fara lock: cea cu scorul cel mai mare
                if (best == null) {
                    double bestScor = -1;
                    for (LLResultTypes.DetectorResult d : det) {
                        double s = scor(d.getTargetXDegrees(), d.getTargetArea());
                        if (s > bestScor) { bestScor = s; best = d; }
                    }
                }
                lockSeenMs = now;
                lockTx = best.getTargetXDegrees();
                lockTy = best.getTargetYDegrees();
            } else {
                for (LLResultTypes.DetectorResult d : det) {
                    if (best == null || d.getTargetArea() > best.getTargetArea()) best = d;
                }
            }
            seteazaBila(best.getTargetXDegrees(), best.getTargetYDegrees(), best.getTargetArea());
            return;
        }

        // pipeline de tip Color (threshold galben)
        List<LLResultTypes.ColorResult> col = res.getColorResults();
        if (col != null && !col.isEmpty()) {
            LLResultTypes.ColorResult best = null;
            for (LLResultTypes.ColorResult c : col) {
                if (best == null || c.getTargetArea() > best.getTargetArea()) best = c;
            }
            seteazaBila(best.getTargetXDegrees(), best.getTargetYDegrees(), best.getTargetArea());
        }
    }

    private void seteazaBila(double tx, double ty, double ta) {
        bilaVizibila = true;
        bilaTx = tx;
        bilaTy = ty;
        bilaTa = ta;
    }

    // ---------------- getters ----------------
    public boolean areTag()      { return tagVizibil; }
    public int getTagId()        { return tagId; }
    public double getTagTx()     { return tagTx; }
    public double getTagTy()     { return tagTy; }
    public double getTagTa()     { return tagTa; }

    public boolean areBila()     { return bilaVizibila; }
    public double getBilaTx()    { return bilaTx; }
    public double getBilaTy()    { return bilaTy; }
    public double getBilaTa()    { return bilaTa; }

    public int getPipeline()     { return pipelineCurent; }

    public void stop() {
        limelight.stop();
    }

    public void telemetrie() {
        telemetry.addData("LL pipeline dorit/real", pipelineCurent + " / " + diagPipelineReal);
        telemetry.addData("LL switch ok / nr trimise", diagSwitchOk + " / " + diagSwitchCount);
        telemetry.addData("LL http cod / sdk ok", diagHttpCode + " / " + diagSdkOk);
        telemetry.addData("LL http eroare", diagHttpErr);
        telemetry.addData("LL url", llUrl);
        telemetry.addData("LL status pipeline", limelight.getStatus() != null ? limelight.getStatus().getPipelineIndex() : -1);
        telemetry.addData("LL running", limelight.isRunning());
        telemetry.addData("LL connected / ms de la ultimul update", limelight.isConnected() + " / " + limelight.getTimeSinceLastUpdate());
        telemetry.addData("LL rezultat", diagNull ? "NULL" : (diagValid ? "valid" : "INVALID"));
        if (pipelineCurent == PIPELINE_BILE) {
            telemetry.addData("LL detectii", diagNrDetectii);
            if (scorAlegere) telemetry.addData("LL scoruri polen", diagScoruri);
            telemetry.addData("Bila", bilaVizibila);
            if (bilaVizibila) {
                telemetry.addData("Bila tx", bilaTx);
                telemetry.addData("Bila ty", bilaTy);
                telemetry.addData("Bila ta", bilaTa);
            }
        } else {
            telemetry.addData("Tag", tagVizibil);
            if (tagVizibil) {
                telemetry.addData("Tag id", tagId);
                telemetry.addData("Tag tx", tagTx);
                telemetry.addData("Tag ty", tagTy);
                telemetry.addData("Tag ta", tagTa);
            }
        }
    }
}
