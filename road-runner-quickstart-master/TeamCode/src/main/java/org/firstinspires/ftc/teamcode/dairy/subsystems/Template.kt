package org.firstinspires.ftc.teamcode.dairy.subsystems

import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import java.lang.annotation.Inherited

class Template private constructor() : Subsystem {
    @Retention(AnnotationRetention.RUNTIME)
    @Target(AnnotationTarget.CLASS)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and SingleAnnotation(
        Attach::class.java)

    override fun preUserInitHook(opMode: Wrapper) {
        defaultCommand = command()
    }

    override fun postUserInitHook(opMode: Wrapper) {
        val hwmap = opMode.opMode.hardwareMap
    }

    companion object {
        val INSTANCE: Template = Template()

        fun function() {
        }

        fun command(): Lambda {
            return Lambda("simple")
                .addRequirements(INSTANCE)
                .setExecute { function() }
        }
    }
}