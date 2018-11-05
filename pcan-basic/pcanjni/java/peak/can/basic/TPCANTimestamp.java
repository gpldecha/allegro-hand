package peak.can.basic;

/**
 * Defines the point of time at which a CAN message was received.
 *
 * @version 1.8
 * @LastChange $Date: 2016-05-13 14:54:19 +0200 (ven. 13 mai 2016) $
 * @author Jonathan Urban/Uwe Wilhelm
 *
 * @Copyright (C) 1999-2014  PEAK-System Technik GmbH, Darmstadt
 * more Info at http://www.peak-system.com
 */
public class TPCANTimestamp
{
    private int millis;
    private short millis_overflow;
    private short micros;

    /**
     * Default constructor
     */
    public TPCANTimestamp()
    {
    }

    /**
     * Gets microseconds
     * @return microseconds (0-999)
     */
    public short getMicros()
    {
        return micros;
    }

    /**
     * Sets microseconds
     * @param micros microseconds (0-999)
     */
    public void setMicros(short micros)
    {
        this.micros = micros;
    }

    /**
     * Gets milliseconds
     * @return milliseconds
     */
    public int getMillis()
    {
        return millis;
    }

    /**
     * Sets milliseconds
     * @param millis milliseconds
     */
    public void setMillis(int millis)
    {
        this.millis = millis;
    }

     /**
     * Gets milliseconds overflow
     * @return milliseconds overflow
     */
    public short getMillis_overflow()
    {
        return millis_overflow;
    }

    /**
     * Sets milliseconds overflow
     * @param millis_overflow milliseconds overflow
     */
    public void setMillis_overflow(short millis_overflow)
    {
        this.millis_overflow = millis_overflow;
    }
}
