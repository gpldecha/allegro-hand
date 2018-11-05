package peak.can.basic;

/**
 * Represents a PCAN filter mode.
 *
 * @version 1.9
 * @LastChange $Date: 2016-09-08 11:34:11 +0200 (jeu. 08 sept. 2016) $
 * @author Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 *
 * @Copyright (C) 1999-2012 PEAK-System Technik GmbH, Darmstadt more Info at
 * http://www.peak-system.com
 */
public enum TPCANMode {

    /**
     * Mode is Standard (11-bit identifier).
     */
    PCAN_MODE_STANDARD(TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue()),
    /**
     * Mode is Extended (29-bit identifier).
     */
    PCAN_MODE_EXTENDED(TPCANMessageType.PCAN_MESSAGE_EXTENDED.getValue());

    private TPCANMode(byte value) {
        this.value = value;
    }

    /**
     * The value of the CAN ID type
     * @return Value of the CAN ID type
     */
    public byte getValue() {
        return this.value;
    }
    private final byte value;
};
