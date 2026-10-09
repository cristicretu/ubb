package DaysBetween;

/**
 * Selected application example (option 3 in the lab brief):
 * Library Borrowing / Return System.
 *
 * For a borrowed book, the library computes how many days the book
 * was kept past the due date and turns that into an overdue fine.
 * Correctly counting days across leap years is essential, hence the
 * dependency on DaysBetween.daysBetween2Dates which itself depends
 * on the leap-year checker.
 *
 * This variant is wired against the STUB-based DaysBetween, so it is
 * exercised by the stub/driver integration tests.
 */
public class LibraryReturnSystem {

    /** Standard fine, in monetary units, per day overdue. */
    public static final double DEFAULT_FINE_PER_DAY = 0.50;

    /** Books returned with more days overdue than this lose lending privileges. */
    public static final long MAX_TOLERATED_OVERDUE_DAYS = 30;

    private final DaysBetween daysBetween;
    private final double finePerDay;

    public LibraryReturnSystem(DaysBetween daysBetween) {
        this(daysBetween, DEFAULT_FINE_PER_DAY);
    }

    public LibraryReturnSystem(DaysBetween daysBetween, double finePerDay) {
        if (daysBetween == null) {
            throw new IllegalArgumentException("daysBetween is required");
        }
        if (finePerDay < 0) {
            throw new IllegalArgumentException("finePerDay must be >= 0");
        }
        this.daysBetween = daysBetween;
        this.finePerDay = finePerDay;
    }

    /**
     * Days the book was kept past the due date. 0 if returned on time
     * or early.
     */
    public long daysOverdue(MyDate dueDate, MyDate returnDate) throws MyValueException {
        long days = daysBetween.daysBetween2Dates(dueDate, returnDate);
        return isStrictlyAfter(returnDate, dueDate) ? days : 0L;
    }

    /** Fine in monetary units. 0 if the book was returned on time or early. */
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
