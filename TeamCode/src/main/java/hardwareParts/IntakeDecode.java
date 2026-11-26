package hardwareParts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import senzori.RGB_indicator;
import senzori.SenzorCuloare;

@Config
public class IntakeDecode {

    private boolean lastHadBall = false;

    private final ElapsedTime ballTimer = new ElapsedTime();
    public static double MIN_BALL_INTERVAL = 0.1;

    private boolean waitingForClear = false;

    // in loc de sleep(100) la FULL
    private final ElapsedTime fullBlinkTimer = new ElapsedTime();
    private boolean fullBlinkDone = false;
    public DcMotorEx motorIntake;
    public Telemetry telemetry;
    public Servo pinionLeft;
    public Servo pinionRight;
    public Servo stopper;
    public SenzorCuloare senzorCuloare;
    public RGB_indicator RGB_indicator;

    public static final double TICKS_PER_REV = 28;
    public static final double GEAR_RATIO = 1;

    public static PIDFCoefficients intakePID = new PIDFCoefficients(0.01, 0.0001, 0.0, 60);

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    public double targetRPM = 0.0;

    public IntakeDecode(HardwareMap hardwaremap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motorIntake = hardwaremap.get(DcMotorEx.class, "motorIntake");
        pinionLeft = hardwaremap.get(Servo.class,"pinionLeft");
        pinionRight = hardwaremap.get(Servo.class,"pinionRight");
        stopper = hardwaremap.get(Servo.class,"stopper");
        senzorCuloare = new SenzorCuloare(hardwaremap, telemetry);
        RGB_indicator = new RGB_indicator(hardwaremap);

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorIntake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intakePID);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        ballTimer.reset();
        fullBlinkTimer.reset();

        telemetry.update();
    }

    public void setIntakeRPM(double rpmOut) {
        targetRPM = rpmOut;
        double rpmMotor = rpmOut / GEAR_RATIO;
        double ticksPerSec = (rpmMotor / 60.0) * TICKS_PER_REV;
        motorIntake.setVelocity(ticksPerSec);
    }

    public void stop() {
        targetRPM = 0;
        motorIntake.setVelocity(0);
        motorIntake.setPower(0);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake_reverse(double rpm) {
        setIntakeRPM(-Math.abs(rpm));
    }

    public void update() {
        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
        motorIntake.setVelocity(targetTicksPerSec);
    }
    public void intakeupdate() {
        motorIntake.setPower(1);
    }
    public void intakeupdateReverse() {
        motorIntake.setPower(-1);
    }

    public void Shoot() {
        setIntakeRPM(4000);
        pinionLeft.setPosition(0.5);
        pinionRight.setPosition(0);
    }
}