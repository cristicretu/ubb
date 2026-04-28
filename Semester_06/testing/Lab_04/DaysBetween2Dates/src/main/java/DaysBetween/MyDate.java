package DaysBetween;

/**
 * Plain immutable Year/Month/Day value, validated at construction.
 *
 * Avoids using java.time on purpose: the whole point of the lab is
 * to compute the number of days between two dates by relying on a
 * separate IsLeapYear component that we then stub or mock.
 *
 * The constructor rejects out-of-range fields AND day numbers that
 * are impossible for the given month/year (e.g. 2023-02-31, 2024-04-31,
 * 2023-02-29). Leap-year February is recognised so that 2024-02-29
 * is accepted while 2023-02-29 is not.
 */
public class MyDate {

    private static final int[] DAYS_IN_MONTH = {
            31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    private final int year;
    private final int month;
    private final int day;

    public MyDate(int year, int month, int day) throws MyValueException {
        if (year < 1) {
            throw new MyValueException("year must be >= 1");
        }
        if (month < 1 || month > 12) {
            throw new MyValueException("month must be in 1..12");
        }
        int max = lastDayOfMonth(month, year);
        if (day < 1 || day > max) {
            throw new MyValueException(
                    "day " + day + " is invalid for "
                            + String.format("%04d-%02d", year, month)
                            + " (allowed range: 1.." + max + ")");
        }
        this.year = year;
        this.month = month;
        this.day = day;
    }

    public int getYear()  { return year;  }
    public int getMonth() { return month; }
    public int getDay()   { return day;   }

    @Override
    public String toString() {
        return String.format("%04d-%02d-%02d", year, month, day);
    }

    /**
     * Last legal day for {@code month} in {@code year}. Encapsulates
     * the leap-year rule locally so MyDate stays self-contained for
     * input validation; the production day-counting in DaysBetween
     * still goes through the real (or stubbed) isLeapYear predicate.
     */
    private static int lastDayOfMonth(int month, int year) {
        if (month == 2 && isLeapYearForValidation(year)) {
            return 29;
        }
        return DAYS_IN_MONTH[month - 1];
    }

    private static boolean isLeapYearForValidation(int year) {
        if ((year % 400) == 0) return true;
        if ((year % 100) == 0) return false;
        return (year % 4) == 0;
    }
}
