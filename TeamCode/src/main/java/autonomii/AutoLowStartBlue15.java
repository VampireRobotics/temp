package autonomii;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import hardwareParts.CalculateTargetPower;
import hardwareParts.IntakeDecode;
import hardwareParts.Sasiu;
import hardwareParts.ShooterPIDPower;
import hardwareParts.turelaodometrieNew;
import localization.PedroLocalization;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import util.SharedAutoData;

@Config
@Autonomous(name="AutoLowStartBlue15")
public class AutoLowStartBlue15 extends OpMode {

    public PedroLocalization pedroLocalizationteleop;

    public IntakeDecode intake;
    public ShooterPIDPower shooter;
    public CalculateTargetPower calcPower;
    private PedroLocalization localization;

    // ✅ NEW turret class (like high autos)
    public turelaodometrieNew turela;

    public Sasiu sasiu;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public static double shootLow = 0.6;          // kept (not used for power now)
    public static double shootintermediar = 0.4;
    public static double offsetx = 0;
    public static double offsety = 0;

    // ✅ set target like TeleOp/High Blue (edit if your low target is different)
    public static double TARGET_X_BLUE = 16.953271028037381;
    public static double TARGET_Y_BLUE = 132;

    // ENTRY FLAG
    private boolean stateEntered = false;
    private boolean arrivedAtShoot = false;

    // save pose once
    private boolean savedPoseToShared = false;

    public enum PathState {
        PREPARE_SHOOTER,
        SHOOT_PRELOAD,
        COOL_DOWN_SHOOTER,
        SPIKEPRIM,
        INTAKEPRIM,

        INTAKEPRIMOPEN,
        INTAKEPRIM_SHOOTPOS,
        COOL_DOWN_SHOOTER2,
        COOL_DOWN_SHOOTER3,
        COOL_DOWN_SHOOTER4,
        SPIKEMIJ,
        INTAKEMIJ,
        INTAKEMIJINTERMEDIAR,
        INTAKEMIJINTERMEDIAR2,
        INTAKEMIJ_SHOOTPOS,
        SPIKELOW,
        INTAKELOW,
        INTAKELOW_SHOOTPOS,
        DRIVE_GATE1,
        DRIVE_GATE2,
        DRIVE_GATE_SHOOTPOS,
        DRIVE_GATE3,
        DRIVE_GATE4,
        DRIVE_GATE_SHOOTPOS2,
        COOL_DOWN_SHOOTER5,
        COOL_DOWN_SHOOTER6,

        PARKING,
    }

    PathState pathState;

    // ===== POSES (unchanged) =====
    private final Pose startPose = new Pose(23, 125, Math.toRadians(143));//143
    private final Pose shootPreload = new Pose(54, 82, Math.toRadians(180));//143
    private final Pose spikePrim = new Pose(50, 83, Math.toRadians(180));
    private final Pose intakePrim = new Pose(17, 83, Math.toRadians(180));
    private final Pose intakePrimOpen = new Pose(17,60,Math.toRadians(180));
    private final Pose intakePrimShootPos = new Pose(54, 110, Math.toRadians(180));
    private final Pose spikeMij = new Pose(50, 58, Math.toRadians(180));

    private final Pose intakeMij = new Pose(15, 55, Math.toRadians(180));
    private final Pose intakeMijIntermediar = new Pose(17, 60, Math.toRadians(270));
    private final Pose intakeMijIntermediar2 = new Pose(50, 58, Math.toRadians(180));
    private final Pose intakeMijShootPos = new Pose(54, 82, Math.toRadians(180));

    private final Pose gateIntake1 = new Pose(17, 63, Math.toRadians(180));
    private final Pose gateIntake2 = new Pose(13, 54, Math.toRadians(130));
    private final Pose gateIntakeShootPos = new Pose(54, 82, Math.toRadians(180));
    //    private final Pose spikeLow = new Pose(55, 36, Math.toRadians(180));
//    private final Pose intakeLow = new Pose(17, 34, Math.toRadians(180));
    private final Pose gateIntake3 = new Pose(17, 63, Math.toRadians(180));
    private final Pose gateIntake4 = new Pose(13, 54, Math.toRadians(130));
    private final Pose gateIntakeShootPos2 = new Pose(54, 82, Math.toRadians(180));
    //    private final Pose intakeLowShootPos = new Pose(60, 110, Math.toRadians(180));
    private final Pose parking = new Pose(60, 110, Math.toRadians(180));

    private Path driveShootPreload;
    private Path driveSpikePrim;
    private Path driveSpikePrimIntakePrim;
    private Path driveIntakePrimOpen;
    private Path driveIntakePrimShootPos;
    private Path driveSpikeMij;
    private Path driveSpikeMijIntakeMij;
    private Path driveIntakeMijIntermediar;
    private Path driveIntakeMijIntermediar2;
    private Path driveIntakeMijShootPos;
    private Path driveSpikeLow;
    private Path driveIntakeLow;
    private Path driveIntakeLowShootPos;
    private Path driveGateIntake1;
    private Path driveGateIntake2;
    private Path driveGateIntakeShootPos;
    private Path driveGateIntake3;
    private Path driveGateIntake4;
    private Path driveGateIntakeShootPos2;
    private Path drivePark;

    public void buildPaths() {
        driveShootPreload = line(startPose, shootPreload).linear(startPose, shootPreload);

//        driveSpikePrim = follower.pathBuilder()
//                .addPath(new BezierLine(shootPreload, spikePrim))
//                .setLinearHeadingInterpolation(shootPreload.getHeading(), spikePrim.getHeading())
//                .build();

        driveSpikePrimIntakePrim = curve(shootPreload, new Pose(58, 86), intakePrim).linear(shootPreload, intakePrim);

        driveIntakePrimOpen = line(intakePrim, intakePrimOpen).linear(intakePrim, intakePrimOpen);

        driveIntakePrimShootPos = line(intakePrimOpen, intakePrimShootPos).linear(intakePrimOpen, intakePrimShootPos);

//        driveSpikeMij = follower.pathBuilder()
//                .addPath(new BezierLine(intakePrimShootPos, spikeMij))
//                .setLinearHeadingInterpolation(intakePrimShootPos.getHeading(), spikeMij.getHeading())
//                .build();

        driveSpikeMijIntakeMij = curve(intakePrimShootPos, new Pose(60, 55), intakeMij).linear(intakePrimShootPos, intakeMij);

        driveIntakeMijIntermediar = line(intakeMij, intakeMijIntermediar).linear(intakeMij, intakeMijIntermediar);

//        driveIntakeMijIntermediar2 = follower.pathBuilder()
//                .addPath(new BezierLine(intakeMijIntermediar, intakeMijIntermediar2))
//                .setLinearHeadingInterpolation(intakeMijIntermediar.getHeading(), intakeMijIntermediar2.getHeading())
//                .build();

        driveIntakeMijShootPos = curve(intakeMij, new Pose(60, 55), intakeMijShootPos).linear(intakeMij, intakeMijShootPos);

//        driveSpikeLow = follower.pathBuilder()
//                .addPath(new BezierLine(intakeMijShootPos, spikeLow))
//                .setLinearHeadingInterpolation(intakeMijShootPos.getHeading(), spikeLow.getHeading())
//                .build();

//        driveIntakeLow = follower.pathBuilder()
//                .addPath(new BezierCurve(intakeMijShootPos,new Pose(68,28), intakeLow))
//                .setLinearHeadingInterpolation(intakeMijShootPos.getHeading(), intakeLow.getHeading())
//                .build();
//
//        driveIntakeLowShootPos = follower.pathBuilder()
//                .addPath(new BezierCurve(intakeLow,new Pose(60,28), intakeLowShootPos))
//                .setLinearHeadingInterpolation(intakeLow.getHeading(), intakeLowShootPos.getHeading())
//                .build();
        driveGateIntake1 = line(intakeMijShootPos, gateIntake1).linear(intakeMijShootPos, gateIntake1);

        driveGateIntake2 = line(gateIntake1, gateIntake2).linear(gateIntake1, gateIntake2);

        driveGateIntakeShootPos = line(gateIntake2, gateIntakeShootPos).linear(gateIntake2, gateIntakeShootPos);

        driveGateIntake3 = line(gateIntakeShootPos, gateIntake3).linear(gateIntakeShootPos, gateIntake3);

        driveGateIntake4 = line(gateIntake3, gateIntake4).linear(gateIntake3, gateIntake4);

        driveGateIntakeShootPos2 = line(gateIntake4, gateIntakeShootPos2).linear(gateIntake4, gateIntakeShootPos2);
//
        drivePark = line(gateIntakeShootPos2, parking).linear(gateIntakeShootPos2, parking);
    }

    // TODO Pedro 3.0: Follower nu mai are setMaxPower. Valorile sunt pastrate aici, dar
    // momentan NU limiteaza viteza robotului.
    private void setMaxPower(double power) {
    }

    // ✅ save pose for TeleOp start (SharedAutoData)
    private void savePoseToSharedOnce() {
        if (savedPoseToShared) return;

        Pose p = (localization != null && localization.getPose() != null)
                ? localization.getPose()
                : (follower != null ? follower.pose() : null);

        if (p == null) return;

        SharedAutoData.lastX = p.x();
        SharedAutoData.lastY = p.y();
        SharedAutoData.lastHeadingRad = p.heading();
        SharedAutoData.hasPose = true;

        savedPoseToShared = true;
    }

    // ✅ shooter power ALWAYS from odometry
    private void applyFixedPower() {
        if (shooter != null) {
            shooter.setTargetPower(0.47);
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
        stateEntered = false;
        arrivedAtShoot = false;
    }

    public void statePathUpdate() {
        shooter.enableAutoAngle(true);

        // (optional / harmless) dacă turelaodometrieNew nu are targetdegrees, șterge linia
        // turela.targetdegrees = 138;

        switch (pathState) {

            case PREPARE_SHOOTER:
                if (!stateEntered) {
                    turela.enableSearchOdometrie();
                    follower.follow(driveShootPreload);
                    stateEntered = true;
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (pathTimer.seconds() > 1.7){ //1.7
                    setPathState(PathState.COOL_DOWN_SHOOTER);
                }
                break;

            case COOL_DOWN_SHOOTER:
                if (pathTimer.seconds() > 0.8) {//1
                    turela.disableSearchOdometrie();
                    if(pathTimer.seconds() > 0.9) {
                        setPathState(PathState.INTAKEMIJ);
                    }
                }
                break;

//            case SPIKEMIJ:
//                if (!stateEntered) {
//                    setMaxPower(0.9);
//                    follower.follow(driveSpikeMij);
//                    stateEntered = true;
//                }
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKEMIJ);
//                }
//                break;

            case INTAKEMIJ:
                if (!stateEntered) {
                    setMaxPower(0.8);
                    intake.setIntakeRPM(1600);
                    follower.follow(driveSpikeMijIntakeMij);
                    stateEntered = true;
                }

                if (!follower.isBusy() && pathTimer.seconds() > 2.5) {//2
                    setMaxPower(0.8);
                    intake.setIntakeRPM(1600);
                    setPathState(PathState.INTAKEMIJ_SHOOTPOS);
                }
                break;

//            case INTAKEMIJINTERMEDIAR:
//                if (!stateEntered) {
//                    intake.setIntakeRPM(1500);
//                    setMaxPower(1.0);
//                    follower.follow(driveIntakeMijIntermediar);
//                    stateEntered = true;
//                }
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKEMIJ_SHOOTPOS);
//                }
//                break;

//            case INTAKEMIJINTERMEDIAR2:
//                if (!stateEntered) {
//                    setMaxPower(0.9);
//                    follower.follow(driveIntakeMijIntermediar2);
//                    stateEntered = true;
//                }
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKEMIJ_SHOOTPOS);
//                }
//                break;

            case INTAKEMIJ_SHOOTPOS:
                if (!stateEntered) {
                    intake.intake_reverse(500);
                    turela.enableSearchOdometrie();
                    setMaxPower(1.0);
                    follower.follow(driveIntakeMijShootPos);
                    stateEntered = true;
                }

                if (!arrivedAtShoot && !follower.isBusy()) {
                    arrivedAtShoot = true;
                    pathTimer.reset();
                }

                if (arrivedAtShoot && pathTimer.seconds() > 0.4) {
                    intake.stop();
                    setPathState(PathState.COOL_DOWN_SHOOTER3);
                }
                break;

            case COOL_DOWN_SHOOTER3:
                if (pathTimer.seconds() > 0.7) {
                    turela.disableSearchOdometrie();
                    if(pathTimer.seconds() > 0.9) {
                        setPathState(PathState.DRIVE_GATE1);
                    }
                }
                break;

//            case SPIKEPRIM:
//                if (!stateEntered) {
//                    setMaxPower(0.8);
//                    follower.follow(driveSpikePrim);
//                    stateEntered = true;
//                }
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKEPRIM);
//                }
//                break;

//            case SPIKELOW:
//                if (!stateEntered) {
//                    setMaxPower(0.9);
//                    follower.follow(driveSpikeLow);
//                    stateEntered = true;
//                }
//
//                if (!follower.isBusy()) {
//                    intake.setIntakeRPM(1100);
//                    setPathState(PathState.INTAKELOW);
//                }
//                break;

//            case INTAKELOW:
//                if (!stateEntered) {
//                    intake.setIntakeRPM(1100);
//                    setMaxPower(0.8);
//                    follower.follow(driveIntakeLow);
//                    stateEntered = true;
//                }
//
//                if (!follower.isBusy() && pathTimer.seconds() > 2.5) {
//                    setMaxPower(0.8);
//                    intake.setIntakeRPM(1100);
//                    sortator.shootPosition();
//                    setPathState(PathState.INTAKELOW_SHOOTPOS);
//                }
//                break;
//
//            case INTAKELOW_SHOOTPOS:
//                if (!stateEntered) {
//                    intake.intake_reverse(500);
//                    turela.enableSearchOdometrie();
//                    sortator.shootPosition();
//                    setMaxPower(1.0);
//                    follower.follow(driveIntakeLowShootPos);
//                    stateEntered = true;
//                }
//
//                if (!arrivedAtShoot && !follower.isBusy()) {
//                    arrivedAtShoot = true;
//                    pathTimer.reset();
//                }
//
//                if (arrivedAtShoot && pathTimer.seconds() > 0.5) {
//                    intake.stop();
//                    sortator.sortatorContinuousRPM(210);
//                    setPathState(PathState.COOL_DOWN_SHOOTER4);
//                }
//                break;
//
//            case COOL_DOWN_SHOOTER4:
//                if (pathTimer.seconds() > 0.3) {
//                    turela.disableSearchOdometrie();
//                    sortator.stop();
//                    sortator.home();
//                    setPathState(PathState.PARKING);
//                }
//                break;
            case DRIVE_GATE1:
                if (!stateEntered) {
                    setMaxPower(0.9);
                    intake.setIntakeRPM(1600);
                    follower.follow(driveGateIntake1);
                    stateEntered = true;
                }

                if (!follower.isBusy()) {
                    intake.setIntakeRPM(1600);
                    setPathState(PathState.DRIVE_GATE2);
                }
                break;

            case DRIVE_GATE2:
                if (!stateEntered) {
                    intake.setIntakeRPM(1600);
                    setMaxPower(0.9);
                    follower.follow(driveGateIntake2);
                    stateEntered = true;
                }

                if(pathTimer.seconds() > 2.3) {
                    setMaxPower(0.9);
                    intake.setIntakeRPM(1600);
                    setPathState(PathState.DRIVE_GATE_SHOOTPOS);
                }
                break;

            case DRIVE_GATE_SHOOTPOS:
                if (!stateEntered) {
                    intake.intake_reverse(500);
                    turela.enableSearchOdometrie();
                    setMaxPower(1.0);
                    follower.follow(driveGateIntakeShootPos);
                    stateEntered = true;
                }

                if (!arrivedAtShoot && !follower.isBusy()) {
                    arrivedAtShoot = true;
                    pathTimer.reset();
                }

                if (arrivedAtShoot && pathTimer.seconds() > 0.4) {//0.8
                    intake.stop();
                    setPathState(PathState.COOL_DOWN_SHOOTER4);
                }
                break;

            case COOL_DOWN_SHOOTER4:
                if (pathTimer.seconds() > 0.8) {
                    turela.disableSearchOdometrie();
                    if(pathTimer.seconds() > 0.9) {
                        setPathState(PathState.DRIVE_GATE3);
                    }
                }
                break;

            case DRIVE_GATE3:
                if (!stateEntered) {
                    setMaxPower(0.9);
                    intake.setIntakeRPM(1600);
                    follower.follow(driveGateIntake3);
                    stateEntered = true;
                }

                if (!follower.isBusy()) {
                    intake.setIntakeRPM(1600);
                    setPathState(PathState.DRIVE_GATE4);
                }
                break;

            case DRIVE_GATE4:
                if (!stateEntered) {
                    intake.setIntakeRPM(1600);
                    setMaxPower(0.9);
                    follower.follow(driveGateIntake4);
                    stateEntered = true;
                }

                if(pathTimer.seconds() > 2.3) {
                    setMaxPower(0.9);
                    intake.setIntakeRPM(1600);
                    setPathState(PathState.DRIVE_GATE_SHOOTPOS2);
                }
                break;

            case DRIVE_GATE_SHOOTPOS2:
                if (!stateEntered) {
                    intake.intake_reverse(500);
                    turela.enableSearchOdometrie();
                    setMaxPower(1.0);
                    follower.follow(driveGateIntakeShootPos2);
                    stateEntered = true;
                }

                if (!arrivedAtShoot && !follower.isBusy()) {
                    arrivedAtShoot = true;
                    pathTimer.reset();
                }

                if (arrivedAtShoot && pathTimer.seconds() > 0.4) {
                    intake.stop();
                    setPathState(PathState.COOL_DOWN_SHOOTER5);
                }
                break;

            case COOL_DOWN_SHOOTER5:
                if (pathTimer.seconds() > 0.8) {
                    turela.disableSearchOdometrie();
                    if(pathTimer.seconds() > 0.9) {
                        setPathState(PathState.INTAKEPRIM);
                    }
                }
                break;

            case INTAKEPRIM:
                if (!stateEntered) {
                    setMaxPower(0.8);
                    intake.setIntakeRPM(1600);
                    follower.follow(driveSpikePrimIntakePrim);
                    stateEntered = true;
                }

                if (!follower.isBusy() && pathTimer.seconds() > 2) {//2.5
                    setMaxPower(0.8);
                    setPathState(PathState.INTAKEPRIM_SHOOTPOS);
                }
                break;

//            case INTAKEPRIMOPEN:
//                if(!stateEntered) {
//                    setMaxPower(1.0);
//                    follower.follow(driveIntakePrimOpen);
//                    stateEntered = true;
//                }
//
//                if(!follower.isBusy() && pathTimer.seconds() > 1) {
//                    setMaxPower(1.0);
//                    setPathState(PathState.INTAKEPRIM_SHOOTPOS);
//                }

            case INTAKEPRIM_SHOOTPOS:
                if (!stateEntered) {
                    intake.intake_reverse(500);
                    turela.enableSearchOdometrie();
                    setMaxPower(0.9);
                    follower.follow(driveIntakePrimShootPos);
                    stateEntered = true;
                }

                if (!arrivedAtShoot && !follower.isBusy()) {
                    arrivedAtShoot = true;
                    pathTimer.reset();
                }

                if (arrivedAtShoot && pathTimer.seconds() > 0.4) {
                    intake.stop();
                    setPathState(PathState.COOL_DOWN_SHOOTER2);
                }
                break;

            case COOL_DOWN_SHOOTER2:
                if (pathTimer.seconds() > 0.8) {
                    turela.disableSearchOdometrie();
                    if(pathTimer.seconds() > 0.9) {
                        setPathState(PathState.PARKING);
                    }
                }
                break;

            case PARKING:
                if (!stateEntered) {
                    setMaxPower(1.0);
                    follower.follow(drivePark);
                    stateEntered = true;
                }

                if (stateEntered && !follower.isBusy()) {
                    shooter.init();
                    turela.init();
                    savePoseToSharedOnce();
                }
                break;

            default:
                telemetry.addLine("Nu merge:)");
                break;
        }
    }

    @Override
    public void init() {

        pathState = PathState.PREPARE_SHOOTER;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.create(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(startPose);
        follower.update();

        localization = new PedroLocalization(follower);

        intake = new IntakeDecode(hardwareMap, telemetry);

        // ✅ calcPower + shooter that uses it (like high)
        calcPower = new CalculateTargetPower();
        shooter = new ShooterPIDPower(hardwareMap, telemetry, calcPower);

        sasiu = new Sasiu(hardwareMap, telemetry);

        // ✅ NEW turret + calcPower link + pose provider
        turela = new turelaodometrieNew(hardwareMap);
        turela.setCalculateTargetPower(calcPower);

        turela.setPoseProvider(new turelaodometrieNew.PoseProvider() {
            @Override public boolean isReady() {
                return localization != null && localization.getPose() != null;
            }
            @Override public double getHeadingRad() {
                return localization.getHeadingRad();
            }
            @Override public double getX() {
                Pose p = (localization != null) ? localization.getPose() : null;
                return (p != null) ? p.x() : 0.0;
            }
            @Override public double getY() {
                Pose p = (localization != null) ? localization.getPose() : null;
                return (p != null) ? p.y() : 0.0;
            }
        });

        // ✅ set target like high/teleop
        turela.targetx = TARGET_X_BLUE;
        turela.targety = TARGET_Y_BLUE;

        shooter.init();
        shooter.on();                  // keep ON
        shooter.enableAutoAngle(true); // auto angle ON

        turela.init();


        buildPaths();

        stateEntered = false;
        arrivedAtShoot = false;
        savedPoseToShared = false;

        // you can start with turret disabled; state machine enables it when needed
        turela.disableSearchOdometrie();

        telemetry.addLine("Auto init OK (ODO turret + ODO power)");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        if (localization != null) localization.update();

        // ✅ turret updates calcPower
        turela.loop();

        applyFixedPower();

        statePathUpdate();

        shooter.read();
        shooter.update();
        shooter.write();


        Pose p = (localization != null && localization.getPose() != null)
                ? localization.getPose()
                : follower.pose();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", "%.2f", p.x());
        telemetry.addData("y", "%.2f", p.y());
        telemetry.addData("heading(deg)", "%.2f", Math.toDegrees(p.heading()));

        telemetry.addData("calcDist", "%.2f", calcPower.getDistance());
        telemetry.addData("calcPower(filt)", "%.3f", calcPower.getPower());

        telemetry.addData("path time", "%.2f", pathTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        savePoseToSharedOnce();
    }
}