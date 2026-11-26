package localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import hardwareParts.turelaodometrie;
import util.SharedAutoData;

public class PedroLocalization {

    private final Follower follower;
    private Telemetry telemetry;   // optional

    private Pose pose = Pose.zero();

    private turelaodometrie turela = null;

    // ===== OFFSET (deg) =====
    private double startHeadingDeg = 0.0;

    // ===== HEADING =====
    public double headingRad = 0.0;
    public double currentx = 0.0;
    public double currenty = 0.0;
    private double headingDeg = 0.0;

    // ✅ dacă e true, NU mai suprascriem startHeadingDeg din turela
    private boolean lockStartHeading = false;

    public PedroLocalization(Follower follower) {
        this.follower = follower;

        // ✅ dacă ai pose din auto, ia heading-ul ca offset 0 (nu mai face add random)
        // Cel mai safe: pornești cu offset 0 și folosești direct follower heading.
        startHeadingDeg = 0.0;
        lockStartHeading = false;
    }

    public void setTurela(turelaodometrie turela) {
        this.turela = turela;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        if (follower == null) return;

        follower.update();
        pose = follower.pose();

        if (pose != null) {
            // ✅ nu mai rescrie offset-ul dacă e blocat (după resetPose)
            if (!lockStartHeading && turela != null) {
                startHeadingDeg = turela.startangle;
            }

            currentx = pose.x();
            currenty = pose.y();
            headingRad = pose.heading();
            headingDeg = Math.toDegrees(headingRad) + startHeadingDeg;
        }
    }

    // ✅ RESET POSE (chemat din TeleOp la Circle)
    public void resetPose(Pose newPose) {
        if (follower == null || newPose == null) return;

        follower.setPose(newPose);
        pose = newPose;

        // după reset, vrem headingDeg == newPose heading (fără offset care îl strică)
        startHeadingDeg = 0.0;
        lockStartHeading = true;

        headingRad = newPose.heading();
        headingDeg = Math.toDegrees(headingRad);
        currentx = newPose.x();
        currenty = newPose.y();
    }

    // dacă vrei să lași iar turela să seteze offset-ul
    public void unlockStartHeading() {
        lockStartHeading = false;
    }

    public Pose getPose() { return pose; }
    public double getHeadingRad() { return headingRad; }
    public double getHeadingDeg() { return headingDeg; }
    public double getStartHeadingDeg() { return startHeadingDeg; }
}
