package team6072.robot2020.utility.logging;

import java.util.logging.*;

import edu.wpi.first.wpilibj.DriverStation;


/**
 * Implementation of logging using the Java logging infrastructure
 */
public class JLogWrapper {

    private Logger mLog;
    private DriverStation mDS;

    // See here for a bunch of useful code
    //  https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/EventLogging.java

    /**
     * Wrap an instance of java.util.logging.Logger with more useful logging methods
     * The properties for the java logger are set in deploy\logging.properties
     * 
     * https://docs.oracle.com/en/java/javase/11/docs/api/java.logging/java/util/logging/Logger.html
     * 
     * Create a logger in a class with:
     *     private static final JLogWrapper mLog = new LogWrapper(DriveSys.class.getName());
     *     private static final PeriodicLogger mPLog = new PeriodicLogger(mLog, 5);
     * Example of use:
     *      mLog.debug("DS.initDriveDist: current yaw: %.3f", NavXSys.getInstance().getYawHeading());
     * 
     * @param logClass
     */
    public JLogWrapper(String logClass) {
        mLog = Logger.getLogger(logClass);
        mDS = DriverStation.getInstance();
    }


    public void info(String msg) {
        if (mLog.isLoggable(Level.INFO)) {
            mLog.log(Level.INFO, msg);
        }
    }

    /**
     * See here for full doco on String.format()
     *      https://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
     * Examples:
     *      String name="team";  
     *      String sf1=String.format("name is %s", name);  
     *      String sf2=String.format("value is %f", 32.33434);          // value is 32.334340
     *      String sf3=String.format("value is %32.12f", 32.33434);     //                  32.334340000000
     * String format:
     *      %[argument_index$][flags][width][.precision]conversion
     * @param msg   message in String.format() format
     * @param params
     */
    public void info(String msg, Object... params) {
        if (mLog.isLoggable(Level.INFO)) {
            mLog.log(Level.INFO, String.format(msg, params));
        }
    }


    /**
     * Message format allows the use of {} formats
     *      https://docs.oracle.com/javase/7/docs/api/java/text/MessageFormat.html
     * 
     * MessageFormat uses patterns of the following form:
      * MessageFormatPattern:
     *          String
     *          MessageFormatPattern FormatElement String
     * 
     *  FormatElement:
     *          { ArgumentIndex }
     *          { ArgumentIndex , FormatType }
     *          { ArgumentIndex , FormatType , FormatStyle }
     * 
     *  FormatType: one of 
     *          number date time choice
     * 
     *  FormatStyle:
     *          short
     *          medium
     *          long
     *          full
     *          integer
     *          currency
     *          percent
     *          SubformatPattern  number uses DecimalFormat pattern
     * 
     * String result = MessageFormat.format("At {1,time} on {1,date}, there was {2} on planet {0,number,integer}.",
     *                             planet, new Date(), event);
     * 
     * Number format characters:
     * 
     * Symbol	Location	Localized?	Meaning
     *  0	    Number	    Yes	        Digit
     *  #	    Number	    Yes	        Digit, zero shows as absent
     *  .	    Number	    Yes	        Decimal separator or monetary decimal separator
     *  -	    Number	    Yes	        Minus sign
     *  ,	    Number	    Yes	        Grouping separator
     *  E	    Number	    Yes	        Separates mantissa and exponent in scientific notation. Need not be quoted in prefix or suffix.
     *  ;	    Subpattern boundary	Yes	Separates positive and negative subpatterns
     *  %	Prefix or suffix	Yes	    Multiply by 100 and show as percentage
     *  \u2030	Prefix or suffix	Yes	Multiply by 1000 and show as per mille value
     *  ¤ (\u00A4)	Prefix or suffix	No	Currency sign, replaced by currency symbol. If doubled, replaced by international currency symbol. If present in a pattern, the monetary decimal separator is used instead of the decimal separator.
     *  '	Prefix or suffix	No	Used to quote special characters in a prefix or suffix, for example, "'#'#" formats 123 to "#123". To create a single quote itself, use two in a row: "# o''clock".
     * 
     * @param msg   message in a java.text.MessageFormat format
     * @param params
     */
    public void infomf(String msg, Object... params) {
        if (mLog.isLoggable(Level.INFO)) {
            mLog.log(Level.INFO, msg, params);
        }
    }



    public void debug(String msg) {
        if (!mDS.isDisabled() && mLog.isLoggable(Level.FINE)) {
            mLog.log(Level.FINE, msg);
        }
    }

    /**
     * Debug logging using MessageFormat string style
     *  "Value is: {0, number} and second: {1,number}"
     * @param msg
     * @param params
     */
    public void debugmf(String msg, Object... params) {
        if (!mDS.isDisabled() && mLog.isLoggable(Level.FINE)) {
            mLog.log(Level.FINE, msg, params);
        }
    }

    public void debug(String msg, Object... params) {
        if (!mDS.isDisabled() && mLog.isLoggable(Level.FINE)) {
            mLog.log(Level.FINE, String.format(msg, params));
        }
    }


    public void severe(String msg, Object... params) {
        if (mLog.isLoggable(Level.SEVERE)) {
            mLog.log(Level.SEVERE, String.format(msg, params));
        }
    }

    public void severe(Throwable t, String msg, Object... params) {
        if (mLog.isLoggable(Level.SEVERE)) {
            mLog.log(Level.SEVERE, "*****************************************************************************");
            mLog.log(Level.SEVERE, String.format(msg + "\n" + exceptionToString(t), params), t);
            mLog.log(Level.SEVERE, "*****************************************************************************");
        }
    }


    /**
     * Create a String representation of an Exception.
     * 
     * @param t
     * @return
     */
    public static final String exceptionToString(Throwable t) {
        final StackTraceElement[] stackTrace = t.getStackTrace();
        final StringBuilder message = new StringBuilder();
        final String padding = "--    ";
        final Throwable cause = t.getCause();

        message.append(padding).append("Exception: ").append(t.getClass().getName()).append('\n');
        message.append(padding).append("Message: ").append(t.getMessage()).append('\n');
        message.append(padding).append('\n');
        for (int i = 0; i < stackTrace.length; i++) {
            message.append(padding).append(stackTrace[i]).append('\n');
        }

        if (cause != null) {
            final StackTraceElement[] causeTrace = cause.getStackTrace();
            message.append(padding).append("Caused by ").append(cause.getClass().getName()).append('\n');
            message.append(padding).append("Because: ").append(cause.getMessage()).append('\n');
            message.append(padding).append(causeTrace[0]).append('\n');
            message.append(padding).append(causeTrace[2]).append('\n');
            message.append(padding).append(causeTrace[3]);
        }

        return message.toString();
    }


}