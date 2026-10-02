package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import hardwareParts.absoluteEncoder;

@TeleOp(name = "Test Absolute Encoder")
public class testAbsoluteEncoder extends LinearOpMode {

    public DcMotor motor1 = null;

    private static final double kP = 0.015;

    @Override
    public void runOpMode() {
        absoluteEncoder encoder = new absoluteEncoder(hardwareMap);
        motor1 = hardwareMap.get(DcMotor.class, "motorIntake");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // merge la 22 grade la init
        while (!isStarted() && !isStopRequested()) {
            encoder.update();
            double error = shortestAngularError(encoder.getAngle(), 22.0);
            motor1.setPower(Math.max(-1.0, Math.min(1.0, kP * error)));
            telemetry.addData("Init - Current", "%.1f deg", encoder.getAngle());
            telemetry.addData("Init - Error",   "%.1f deg", error);
            telemetry.update();
        }
        motor1.setPower(0);

        waitForStart();

        double targetAngle = 22.0;

        while (opModeIsActive()) {
            encoder.update();

            if (gamepad1.cross) {
                targetAngle = normalizeAngle(targetAngle + 180.0);
            }

            else if (gamepad1.square) {
                targetAngle = normalizeAngle(targetAngle + 270.0);
            }

            double error = shortestAngularError(encoder.getAngle(), targetAngle);
            double power = Math.max(-1.0, Math.min(1.0, kP * error));
            motor1.setPower(power);

            telemetry.addData("Target",  "%.1f deg", targetAngle);
            telemetry.addData("Current", "%.1f deg", encoder.getAngle());
            telemetry.addData("Error",   "%.1f deg", error);
            telemetry.addData("Power",   "%.3f",     power);
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double shortestAngularError(double current, double target) {
        double error = target - current;
        while (error >  180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}