package DaysBetween;

/**
 * Counterpart of LongSeq_VerifyIsPrimeMock from the StubsIsPrime
 * example: same logic as DaysBetween#daysBetween2Dates but the
 * leap-year predicate is delegated to an injected VerifyIsLeapYear
 * instance, so Mockito can stub the predicate at call sites.
 */
public class DaysBetween_VerifyIsLeapYearMock {

    private static final int[] DAYS_IN_MONTH = {
            31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    private final VerifyIsLeapYear verifyIsLeapYear;

    public DaysBetween_VerifyIsLeapYearMock(VerifyIsLeapYear verifyIsLeapYear) {
        this.verifyIsLeapYear = verifyIsLeapYear;
        System.out.println("DaysBetween (mock-based) ...");
    }

    private long daysInYear(int year) throws MyValueException {
        return verifyIsLeapYear.isLeapYear(year) ? 366 : 365;
    }

    /** Days from January 1st of d.year (inclusive) to d (exclusive). */
    long daysFromYearStart(MyDate d) throws MyValueException {
        long days = 0;
        for (int m = 1; m < d.getMonth(); m++) {
            days += DAYS_IN_MONTH[m - 1];
            if (m == 2 && verifyIsLeapYear.isLeapYear(d.getYear())) {
                days += 1;
            }
        }
        days += d.getDay() - 1;
        return days;
    }

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
