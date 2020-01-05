package frc.lib;

import org.omg.CORBA.Any;

public class CLogger {
    // Logger Mode Enum
    public enum cLoggerMode {
        COMPETITION,
        PRACTICE,
        TESTING,
        DEVELOPMENT
    }

    // The log type
    public static enum cLogType {
        MajorError,
        MinorError,
        Warning,
        Debug,
        Undefined
    }

    // Should Logs
    private static boolean sl_majorErrors;
    private static boolean sl_minorErrors;
    private static boolean sl_warnings;
    private static boolean sl_debugPrints;
    private static boolean sl_undefinedPrints;
    
    /**
     * Constructor 
     * @param mode Logger Mode
     */
    public CLogger(cLoggerMode mode) {
        setup(mode);
    }

    /**
     * Catch for constructor missing params
     */
    public CLogger() {
        setup(cLoggerMode.DEVELOPMENT);
    }

    /**
     * Initial setup called by the constructor
     * @param mode Passed by constructor
     */
    private static void setup(cLoggerMode mode) {
        switch (mode) {
            case COMPETITION:
                sl_majorErrors     = false;
                sl_minorErrors     = false;
                sl_warnings        = false;
                sl_debugPrints     = false;
                sl_undefinedPrints = false;
                break;
            case PRACTICE:
                sl_majorErrors     = true;
                sl_minorErrors     = false;
                sl_warnings        = false;
                sl_debugPrints     = false;
                sl_undefinedPrints = false;
                break;
            case TESTING:
                sl_majorErrors     = true;
                sl_minorErrors     = true;
                sl_warnings        = true;
                sl_debugPrints     = false;
                sl_undefinedPrints = false;
                break;
            case DEVELOPMENT:
                sl_majorErrors     = true;
                sl_minorErrors     = true;
                sl_warnings        = true;
                sl_debugPrints     = true;
                sl_undefinedPrints = true;
                break;
            default:
                break;
        }
    }


    /**
     * This is the public logging manager
     * @param value Value to log
     * @param type What type of log is this
     */
    public static void log(Any value, cLogType type) {
        switch (type) {
            case MajorError:
                if (sl_majorErrors) {
                    System.out.print(value);
                }
                break;
            case MinorError:
                if (sl_minorErrors) {
                    System.out.print(value);
                }
                break;
            case Warning:
                if (sl_warnings) {
                    System.out.print(value);
                }
                break;
            case Debug:
                if (sl_debugPrints) {
                    System.out.print(value);
                }
                break;
            case Undefined:
                if (sl_undefinedPrints) {
                    System.out.print(value);
                }
                break;
            default:
                if (sl_undefinedPrints) {
                    System.out.print(value);
                }
                break;
        }
    }

    /**
     * A catch for an undefined log type
     * @param value Value to log
     */
    public static void log(Any value) {
        log(value, cLogType.Undefined);
    }

    /**
     * The replicative method of log, but instead uses println
     * @param value The value to log
     * @param type The type of value to log
     */
    public static void logln(Any value, cLogType type) {
        switch (type) {
            case MajorError:
                if (sl_majorErrors) {
                    System.out.println(value);
                }
                break;
            case MinorError:
                if (sl_minorErrors) {
                    System.out.println(value);
                }
                break;
            case Warning:
                if (sl_warnings) {
                    System.out.println(value);
                }
                break;
            case Debug:
                if (sl_debugPrints) {
                    System.out.println(value);
                }
                break;
            case Undefined:
                if (sl_undefinedPrints) {
                    System.out.println(value);
                }
                break;
            default:
                if (sl_undefinedPrints) {
                    System.out.println(value);
                }
                break;
        }
    }

    /**
     * A catch for an undefined logType for logln
     * @param value The value to log
     */
    public static void logln(Any value) {
        logln(value, cLogType.Undefined);
    }

    /**
     * Bypasses the logging manager and goes straight to sysout
     * @param value The value to log
     */
    public static void logBypass(Any value) {
        System.out.print(value);
    }

    /**
     * Bypasses the logging manager and does a println
     * @param value The value to log
     */
    public static void logBypassln(Any value) {
        System.out.println(value);
    }
}