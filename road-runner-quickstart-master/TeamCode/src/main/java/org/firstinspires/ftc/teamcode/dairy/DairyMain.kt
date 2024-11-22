package org.firstinspires.ftc.teamcode.dairy

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.dairy.pasteurized.layering.LayeredGamepad
import dev.frozenmilk.dairy.pasteurized.layering.MapLayeringSystem
import dev.frozenmilk.mercurial.Mercurial

enum class GamepadLayers {
    TELEOP,
    ENDGAME,
}

@Mercurial.Attach
class DairyMain: OpMode() {
    /*
    val teleOpGamepad1 = SDKGamepad(gamepad1)
    val endgameGamepad1 = SDKGamepad(gamepad1)

    val teleOpGamepad2 = SDKGamepad(gamepad1)
    val endgameGamepad2 = SDKGamepad(gamepad1)

    val pasteurizedGamepad1 = mapOf(
        GamepadLayers.TELEOP to teleOpGamepad1,
        GamepadLayers.ENDGAME to endgameGamepad1
    )

    val pasteurizedGamepad2 = mapOf(
        GamepadLayers.TELEOP to teleOpGamepad2,
        GamepadLayers.ENDGAME to endgameGamepad2
    )

    val enumLayeringSystem1 = MapLayeringSystem(GamepadLayers.TELEOP, pasteurizedGamepad1)
    val enumLayeringSystem2 = MapLayeringSystem(GamepadLayers.TELEOP, pasteurizedGamepad1)
    val layeredGamepad1 = LayeredGamepad(enumLayeringSystem1)
    val layeredGamepad2 = LayeredGamepad(enumLayeringSystem2)
    */



    override fun init() {

    }

    override fun loop() {
        var a = Pasteurized.gamepad1.a;


    }

}