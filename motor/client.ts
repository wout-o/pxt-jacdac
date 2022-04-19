namespace modules {
    /**
     * A bi-directional DC motor.
     **/
    //% fixedInstances blockGap=8
    export class MotorClient extends jacdac.Client {
        private readonly _duty: jacdac.RegisterClient<[number]>
        private readonly _enabled: jacdac.RegisterClient<[boolean]>
        private readonly _loadTorque: jacdac.RegisterClient<[number]>
        private readonly _loadSpeed: jacdac.RegisterClient<[number]>
        private readonly _reversible: jacdac.RegisterClient<[boolean]>

        constructor(role: string) {
            super(jacdac.SRV_MOTOR, role)

            this._duty = this.addRegister<[number]>(
                jacdac.MotorReg.Duty,
                jacdac.MotorRegPack.Duty
            )
            this._enabled = this.addRegister<[boolean]>(
                jacdac.MotorReg.Enabled,
                jacdac.MotorRegPack.Enabled
            )
            this._loadTorque = this.addRegister<[number]>(
                jacdac.MotorReg.LoadTorque,
                jacdac.MotorRegPack.LoadTorque,
                jacdac.RegisterClientFlags.Optional |
                    jacdac.RegisterClientFlags.Const
            )
            this._loadSpeed = this.addRegister<[number]>(
                jacdac.MotorReg.LoadSpeed,
                jacdac.MotorRegPack.LoadSpeed,
                jacdac.RegisterClientFlags.Optional |
                    jacdac.RegisterClientFlags.Const
            )
            this._reversible = this.addRegister<[boolean]>(
                jacdac.MotorReg.Reversible,
                jacdac.MotorRegPack.Reversible,
                jacdac.RegisterClientFlags.Optional |
                    jacdac.RegisterClientFlags.Const
            )
        }

        /**
         * PWM duty cycle of the motor. Use negative/positive values to run the motor forwards and backwards.
         * Positive is recommended to be clockwise rotation and negative counterclockwise. A duty of ``0``
         * while ``enabled`` acts as brake.
         */
        //% callInDebugger
        //% group="Motor"
        //% blockId=jacdac_motor_duty___get
        //% weight=100
        duty(): number {
            this.start()
            const values = this._duty.pauseUntilValues() as any[]
            return values[0] * 100
        }

        /**
         * PWM duty cycle of the motor. Use negative/positive values to run the motor forwards and backwards.
         * Positive is recommended to be clockwise rotation and negative counterclockwise. A duty of ``0``
         * while ``enabled`` acts as brake.
         */
        //% group="Motor"
        //% blockId=jacdac_motor_duty___set
        //% weight=99
        //% value.min=-100
        //% value.max=100
        //% value.defl=100
        setDuty(value: number) {
            this.start()
            const values = this._duty.values as any[]
            values[0] = value / 100
            this._duty.values = values as [number]
        }

        /**
         * Turn the power to the motor on/off.
         */
        //% callInDebugger
        //% group="Motor"
        //% block="%motor enabled"
        //% blockId=jacdac_motor_enabled___get
        //% weight=98
        enabled(): boolean {
            this.start()
            const values = this._enabled.pauseUntilValues() as any[]
            return !!values[0]
        }

        /**
         * Turn the power to the motor on/off.
         */
        //% group="Motor"
        //% blockId=jacdac_motor_enabled___set
        //% block="set %motor %value=toggleOnOff"
        //% weight=97
        setEnabled(value: boolean) {
            this.start()
            const values = this._enabled.values as any[]
            values[0] = value ? 1 : 0
            this._enabled.values = values as [boolean]
        }

        /**
         * Torque required to produce the rated power of an electrical motor at load speed.
         */
        //% callInDebugger
        //% group="Motor"
        //% weight=96
        loadTorque(): number {
            this.start()
            const values = this._loadTorque.pauseUntilValues() as any[]
            return values[0]
        }

        /**
         * Revolutions per minute of the motor under full load.
         */
        //% callInDebugger
        //% group="Motor"
        //% weight=95
        loadSpeed(): number {
            this.start()
            const values = this._loadSpeed.pauseUntilValues() as any[]
            return values[0]
        }

        /**
         * Reversible.
         */
        //% callInDebugger
        //% group="Motor"
        //% weight=95
        reversible(): boolean {
            this.start()
            const values = this._loadSpeed.pauseUntilValues() as any[]
            return !!values[0]
        }

        /**
         * Set the throttle on a DC motor
         * @param speed the throttle of the motor from -100% to 100%
         */
        //% group="Motor"
        //% weight=100
        //% blockId=jdmotorrun block="run %motor at $speed=speedPicker \\%"
        //% servo.fieldEditor="gridpicker"
        //% servo.fieldOptions.width=220
        //% servo.fieldOptions.columns=2
        run(speed: number): void {
            speed = Math.clamp(-100, 100, speed)
            if (speed == 0) this.setEnabled(false)
            else {
                this.setDuty(speed)
                this.setEnabled(true)
            }
        }
    }

    //% fixedInstance whenUsed
    export const motor1 = new MotorClient("motor1")
}
