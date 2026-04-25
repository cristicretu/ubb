package DaysBetween;

/**
 * Mock-friendly counterpart of LibraryReturnSystem: the application
 * is wired against DaysBetween_VerifyIsLeapYearMock, which delegates
 * the leap-year check to an injected VerifyIsLeapYear that the tests
 * mock with Mockito.
 */
public class LibraryReturnSystem_Mock {

    public static final double DEFAULT_FINE_PER_DAY = 0.50;
    public static final long MAX_TOLERATED_OVERDUE_DAYS = 30;

    private final DaysBetween_VerifyIsLeapYearMock daysBetween;
    private final double finePerDay;

    public LibraryReturnSystem_Mock(DaysBetween_VerifyIsLeapYearMock daysBetween) {
        this(daysBetween, DEFAULT_FINE_PER_DAY);
    }

    public LibraryReturnSystem_Mock(DaysBetween_VerifyIsLeapYearMock daysBetween, double finePerDay) {
        if (daysBetween == null) {
            throw new IllegalArgumentException("daysBetween is required");
        }
        if (finePerDay < 0) {
            throw new IllegalArgumentException("finePerDay must be >= 0");
        }
        this.daysBetween = daysBetween;
        this.finePerDay = finePerDay;
    }

    public long daysOverdue(MyDate dueDate, MyDate returnDate) throws MyValueException {
        long days = daysBetween.daysBetween2Dates(dueDate, returnDate);
        return isStrictlyAfter(returnDate, dueDate) ? days : 0L;
    }

    public double computeFine(MyDate dueDate, MyDate returnDate) throws MyValueException {
        return daysOverdue(dueDate, returnDate) * finePerDay;
    }

    public boolean shouldRevokePrivileges(MyDate dueDate, MyDate returnDate) throws MyValueException {
        return daysOverdue(dueDate, returnDate) > MAX_TOLERATED_OVERDUE_DAYS;
    }

    private static boolean isStrictlyAfter(MyDate x, MyDate y) {
        if (x.getYear()  != y.getYear())  return x.getYear()  > y.getYear();
        if (x.getMonth() != y.getMonth()) return x.getMonth() > y.getMonth();
        return x.getDay() > y.getDay();
    }
}
