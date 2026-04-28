package DaysBetween;

/**
 * Mirrors the role of LongSeq in the StubsIsPrime example:
 *
 *   - isLeapYear         : the real implementation (the unit).
 *   - isLeapYearStub     : a hand-coded stub for the same predicate,
 *                          used by stub-based driver tests.
 *   - daysBetween2Dates  : the production function. By default it
 *                          uses the real isLeapYear, so it works for
 *                          arbitrary years; in stub mode (opt-in via
 *                          the boolean constructor) it routes the
 *                          predicate through isLeapYearStub instead,
 *                          which is the driver/stub pattern asked for
 *                          by the lab.
 */
public class DaysBetween {

    private static final int[] DAYS_IN_MONTH = {
            31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    /** When true, daysBetween2Dates routes the leap-year check through isLeapYearStub. */
    private final boolean useStub;

    /** Production constructor: daysBetween2Dates uses the real isLeapYear. */
    public DaysBetween() {
        this(false);
    }

    /**
     * Driver constructor: when {@code useStub} is true, daysBetween2Dates
     * routes the leap-year check through the hard-coded
     * {@link #isLeapYearStub(int)} method, so stub-based driver tests
     * can demonstrate the integration-testing-with-stubs pattern.
     */
    public DaysBetween(boolean useStub) {
        this.useStub = useStub;
        System.out.println("DaysBetween ("
                + (useStub ? "stub-based" : "real") + ") ...");
    }

    /**
     * Real implementation of the leap-year rule:
     *   - divisible by 400                               -> leap
     *   - divisible by 100 (but not 400)                 -> common
     *   - divisible by 4 (but not 100)                   -> leap
     *   - otherwise                                      -> common
     */
    public boolean isLeapYear(int year) throws MyValueException {
        if (year < 1) {
            throw new MyValueException("year must be >= 1");
        }
        if ((year % 400) == 0) {
            return true;
        }
        if ((year % 100) == 0) {
            return false;
        }
        return (year % 4) == 0;
    }

    /**
     * Stub for isLeapYear, used by daysBetween2Dates when running
     * in stub mode (driver-style integration tests).
     *
     * The stub answers only for the years that show up in the lab
     * test cases. Any unexpected year throws, so a missing test
     * wiring is loud rather than silently wrong.
     */
    public boolean isLeapYearStub(int year) throws MyValueException {
        if (year < 1) {
            throw new MyValueException("year must be >= 1");
        }
        switch (year) {
            case 2000:
            case 2004:
            case 2008:
            case 2012:
            case 2016:
            case 2020:
            case 2024:
            case 2400:
                return true;
            case 1900:
            case 2001:
            case 2002:
            case 2003:
            case 2005:
            case 2017:
            case 2018:
            case 2019:
            case 2021:
            case 2022:
            case 2023:
            case 2025:
            case 2026:
            case 2100:
            case 2200:
            case 2300:
                return false;
            default:
                throw new MyValueException(
                        "isLeapYearStub: year " + year + " not stubbed");
        }
    }

    /** Routes through the real or the stubbed predicate, depending on {@link #useStub}. */
    private boolean leapYearForCalc(int year) throws MyValueException {
        return useStub ? isLeapYearStub(year) : isLeapYear(year);
    }

    /** Days from January 1st of d.year (inclusive) to d (exclusive). */
    long daysFromYearStart(MyDate d) throws MyValueException {
        long days = 0;
        for (int m = 1; m < d.getMonth(); m++) {
            days += DAYS_IN_MONTH[m - 1];
            if (m == 2 && leapYearForCalc(d.getYear())) {
                days += 1;
            }
        }
        days += d.getDay() - 1;
        return days;
    }

    private long daysInYear(int year) throws MyValueException {
        return leapYearForCalc(year) ? 366 : 365;
    }

    /**
     * Absolute number of days between d1 and d2, computed via the
     * leap-year predicate (real one by default; stub when this
     * instance was created in stub mode).
     */
    public long daysBetween2Dates(MyDate d1, MyDate d2) throws MyValueException {
        if (d1 == null || d2 == null) {
            throw new MyValueException("dates must not be null");
        }
        MyDate a = d1, b = d2;
        if (isAfter(d1, d2)) {
            a = d2;
            b = d1;
        }

        if (a.getYear() == b.getYear()) {
            return daysFromYearStart(b) - daysFromYearStart(a);
        }

        long days = daysInYear(a.getYear()) - daysFromYearStart(a);
        for (int y = a.getYear() + 1; y < b.getYear(); y++) {
            days += daysInYear(y);
        }
        days += daysFromYearStart(b);
        return days;
    }

    private static boolean isAfter(MyDate x, MyDate y) {
        if (x.getYear()  != y.getYear())  return x.getYear()  > y.getYear();
        if (x.getMonth() != y.getMonth()) return x.getMonth() > y.getMonth();
        return x.getDay() > y.getDay();
    }
}
