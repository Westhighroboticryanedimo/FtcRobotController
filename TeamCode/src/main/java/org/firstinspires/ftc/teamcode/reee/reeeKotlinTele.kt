package org.firstinspires.ftc.teamcode.reee

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "reee kotlin tele", group = "reee")

class reeeKotlinTele : Opmode() {
    var motor : DcMotor? = null
    var controller : Controller? = null
    const val TURN_TICKS = 25.95

    var dir = false

    override fun init() {
        motor = hardwareMap.dcMotor.get("motor")
        controller = Controller(gamepad1)
    }

    override fun loop() {
        if (motor.getCurrentPosition() <= 0) {
            dir = true
        } else if (motor.getCurrentPosition() >= TURN_TICKS) {
            dir = false
        }

        if (dir) {
            motor.setPower(0.25)
        } else {
            motor.setPower(-0.25)
        }

        telemetry.addData("ticks:", motor.getCurrentPosition())
        telemetry.update()
    }
}
