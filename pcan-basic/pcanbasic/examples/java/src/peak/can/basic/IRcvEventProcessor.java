package peak.can.basic;

/**
 * This interface is implemented by classes which need to process the CAN Receive-Event.
 *
 * @version 1.8
 * @LastChange $Date: 2016-09-08 11:34:11 +0200 (jeu. 08 sept. 2016) $
 * @author Jonathan Urban/Uwe Wilhelm
 *
 * @Copyright (C) 1999-2014  PEAK-System Technik GmbH, Darmstadt
 * more Info at http://www.peak-system.com
 */
public interface IRcvEventProcessor
{
    /**
     * This method is called by the RcvEventDispatcher to process the CAN Receive-Event
     * by the current implementor
     * @param channel CAN channel to process event
     */
    public void processRcvEvent(TPCANHandle channel);
}