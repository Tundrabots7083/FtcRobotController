package org.firstinspires.ftc.teamcode.components.utilities;

import android.util.Log;



/* This class implements the platform dependent debug logging.
 */
public class HalDbgLog
{
    private static final String TAG = "TrcDbg";

    /* This method is called to print a message with the specified message level to the debug console.
     *
     * @param level specifies the message level.
     * @param msg specifies the message.
     */
    public static void msg(TrcDbgTrace.MsgLevel level, String msg)
    {
        switch (level)
        {
            case FATAL:
            case ERR:
                Log.e(TAG, msg);
                break;

            case WARN:
                Log.w(TAG, msg);
                break;

            case INFO:
                Log.i(TAG, msg);
                break;

            case VERBOSE:
                Log.v(TAG, msg);
                break;
        }
    }   //msg

    /**
     * This method is called to print a message to the debug console.
     *
     * @param msg specifies the message.
     */
    public static void traceMsg(String msg)
    {
        Log.d(TAG, msg);
    }   //traceMsg

}   //class HalDbgLog