package peak.can.basic;

/**
 * Represent the PCAN error and status codes
 *
 * @version 1.9
 * @LastChange $Date: 2016-09-08 11:34:11 +0200 (jeu. 08 sept. 2016) $
 * @author Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 *
 * @Copyright (C) 1999-2012 PEAK-System Technik GmbH, Darmstadt more Info at
 * http://www.peak-system.com
 */
public enum TPCANStatus {

    /**
     * No Error
     */
    PCAN_ERROR_OK(0x00000),
    /**
     * Transmit buffer in CAN controller is full
     */
    PCAN_ERROR_XMTFULL(0x00001),
    /**
     * CAN controller was read too late
     */
    PCAN_ERROR_OVERRUN(0x00002),
    /**
     * Bus error: an error counter reached the 'light' limit
     */
    PCAN_ERROR_BUSLIGHT(0x00004),
    /**
     * Bus error: an error counter reached the 'heavy' limit
     */
    PCAN_ERROR_BUSHEAVY(0x00008),
    /**
     * Bus error: an error counter reached the 'warning' limit
     */
    PCAN_ERROR_BUSWARNING(PCAN_ERROR_BUSHEAVY.value),
    /**
     * Bus error: the CAN controller is error passive
     */
    PCAN_ERROR_BUSPASSIVE(0x40000),
    /**
     * Bus error: the CAN controller is in bus-off state
     */
    PCAN_ERROR_BUSOFF(0x00010),
    /**
     * PCAN_ERROR_ANYBUSERR
     */
    PCAN_ERROR_ANYBUSERR(PCAN_ERROR_BUSWARNING.value | 
            PCAN_ERROR_BUSLIGHT.value | PCAN_ERROR_BUSHEAVY.value | 
            PCAN_ERROR_BUSOFF.value | PCAN_ERROR_BUSPASSIVE.value),
    /**
     * Receive queue is empty
     */
    PCAN_ERROR_QRCVEMPTY(0x00020),
    /**
     * Receive queue was read too late
     */
    PCAN_ERROR_QOVERRUN(0x00040),
    /**
     * Transmit queue is full
     */
    PCAN_ERROR_QXMTFULL(0x00080),
    /**
     * Test of the CAN controller hardware registers failed (no hardware found)
     */
    PCAN_ERROR_REGTEST(0x00100),
    /**
     * Driver not loaded
     */
    PCAN_ERROR_NODRIVER(0x00200),
    /**
     * Hardware already in use by a Net
     */
    PCAN_ERROR_HWINUSE(0x00400),
    /**
     * A Client is already connected to the Net
     */
    PCAN_ERROR_NETINUSE(0x00800),
    /**
     * Hardware handle is invalid
     */
    PCAN_ERROR_ILLHW(0x01400),
    /**
     * Net handle is invalid
     */
    PCAN_ERROR_ILLNET(0x01800),
    /**
     * Client handle is invalid
     */
    PCAN_ERROR_ILLCLIENT(0x01C00),
    /**
     * Mask for all handle errors
     */
    PCAN_ERROR_ILLHANDLE(PCAN_ERROR_ILLHW.value | PCAN_ERROR_ILLNET.value | PCAN_ERROR_ILLCLIENT.value),
    /**
     * Resource (FIFO, Client, timeout) cannot be created
     */
    PCAN_ERROR_RESOURCE(0x02000),
    /**
     * Invalid parameter
     */
    PCAN_ERROR_ILLPARAMTYPE(0x04000),
    /**
     * Invalid parameter value
     */
    PCAN_ERROR_ILLPARAMVAL(0x08000),
    /**
     * Unknow error
     */
    PCAN_ERROR_UNKNOWN(0x10000),
    /**
     * Invalid data, function, or action.
     */
    PCAN_ERROR_ILLDATA(0x20000),
    /**
     * An operation was successfully carried out, however, irregularities were
     * registered
     *
     * @remark Value was changed from 0x40000 to 0x4000000
     */
    PCAN_ERROR_CAUTION(0x2000000),
    /**
     * Channel is not initialized
     */
    PCAN_ERROR_INITIALIZE(0x4000000),
    /**
     * Invalid operation
     *
     * @remark Value was changed from 0x80000 to 0x8000000
     */
    PCAN_ERROR_ILLOPERATION(0x8000000);

    private TPCANStatus(int value) {
        this.value = value;
    }

    /**
     * The value of the CAN status code
     * @return Value of the CAN status code
     */
    public int getValue() {
        return this.value;
    }
    private final int value;
};
