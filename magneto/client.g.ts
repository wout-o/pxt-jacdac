namespace modules {
    /**
     * A 3-axis magnetometer.
     **/
    //% fixedInstances blockGap=8
    export class MagnetoClient extends jacdac.SensorClient<[number,number,number]> {
        constructor(role: string) {
            super(jacdac.SRV_MAGNETO, role, "i32 i32 i32");
        }
    
        /**
        * Indicates the current magnetic field on magnetometer.
        * For reference: `1 mgauss` is `100 nT` (and `1 gauss` is `100 000 nT`).
        */
        //% blockId=jacdacmagnetomer_101_0
        //% group="x"
        //% blockCombine block="x" callInDebugger
        get x(): number {
            const values = this.values();
            return values && values[0];
        }

        /**
        * Indicates the current magnetic field on magnetometer.
        * For reference: `1 mgauss` is `100 nT` (and `1 gauss` is `100 000 nT`).
        */
        //% blockId=jacdacmagnetomer_101_1
        //% group="y"
        //% blockCombine block="y" callInDebugger
        get y(): number {
            const values = this.values();
            return values && values[1];
        }

        /**
        * Indicates the current magnetic field on magnetometer.
        * For reference: `1 mgauss` is `100 nT` (and `1 gauss` is `100 000 nT`).
        */
        //% blockId=jacdacmagnetomer_101_2
        //% group="z"
        //% blockCombine block="z" callInDebugger
        get z(): number {
            const values = this.values();
            return values && values[2];
        }

            
    }

    //% fixedInstance whenUsed
    export const magneto = new MagnetoClient("magneto");
}