package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import hardwareParts.absoluteEncoder;

@TeleOp(name = "Test Absolute Encoder")
public class testAbsoluteEncoder extends LinearOpMode {
public DcMotor motor1 = null;
    @Override
    public void runOpMode() {
        absoluteEncoder encoder = new absoluteEncoder(hardwareMap);
        motor1 = hardwareMap.get(DcMotor.class, "motorIntake");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.cross)
                motor1.setPower(0.5);
            else if(gamepad1.square)
                motor1.setPower(0);
            encoder.update();

            telemetry.addData("Voltage", "%.3f V", encoder.getVoltage());
            telemetry.addData("Angle",   "%.1f deg", encoder.getAngle());
            telemetry.update();
        }
    }
}
