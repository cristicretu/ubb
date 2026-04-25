package DaysBetween;

/**
 * Plain immutable Year/Month/Day value, validated at construction.
 *
 * Avoids using java.time on purpose: the whole point of the lab is
 * to compute the number of days between two dates by relying on a
 * separate IsLeapYear component that we then stub or mock.
 */
public class MyDate {

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
        if (day < 1 || day > 31) {
            throw new MyValueException("day must be in 1..31");
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
}
