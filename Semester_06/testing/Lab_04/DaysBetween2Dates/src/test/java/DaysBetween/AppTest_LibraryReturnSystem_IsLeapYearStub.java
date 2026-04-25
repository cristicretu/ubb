package DaysBetween;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Application-context integration test (Library Borrowing / Return
 * System).
 *
 * The system uses DaysBetween.daysBetween2Dates which itself relies
 * on isLeapYearStub. So this is an end-to-end driver test:
 *   LibraryReturnSystem  -->  DaysBetween.daysBetween2Dates
 *                         -->  DaysBetween.isLeapYearStub
 */
public class AppTest_LibraryReturnSystem_IsLeapYearStub {

    private static final double EPS = 1e-9;

    private LibraryReturnSystem library;

    @Before
    public void setup() {
        library = new LibraryReturnSystem(new DaysBetween(), 0.50);
    }

    @Test
    public void test_returned_on_time_no_fine() throws MyValueException {
        MyDate due    = new MyDate(2024, 3, 10);
        MyDate ret    = new MyDate(2024, 3, 10);

        assertEquals(0L, library.daysOverdue(due, ret));
        assertEquals(0.0, library.computeFine(due, ret), EPS);
        assertFalse(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_returned_early_no_fine() throws MyValueException {
        MyDate due    = new MyDate(2024, 3, 15);
        MyDate ret    = new MyDate(2024, 3, 10);

        assertEquals(0L, library.daysOverdue(due, ret));
        assertEquals(0.0, library.computeFine(due, ret), EPS);
        assertFalse(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_returned_5_days_late() throws MyValueException {
        MyDate due    = new MyDate(2024, 3, 10);
        MyDate ret    = new MyDate(2024, 3, 15);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
        assertFalse(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_returned_late_across_leap_february() throws MyValueException {
        MyDate due    = new MyDate(2024, 2, 28);
        MyDate ret    = new MyDate(2024, 3, 5);

        assertEquals(6L, library.daysOverdue(due, ret));
        assertEquals(3.00, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_returned_late_across_common_february() throws MyValueException {
        MyDate due    = new MyDate(2023, 2, 28);
        MyDate ret    = new MyDate(2023, 3, 5);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_30_days_late_no_revocation() throws MyValueException {
        MyDate due    = new MyDate(2024, 3, 1);
        MyDate ret    = new MyDate(2024, 3, 31);

        assertEquals(30L, library.daysOverdue(due, ret));
        assertEquals(15.00, library.computeFine(due, ret), EPS);
        assertFalse(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_31_days_late_revoke_privileges() throws MyValueException {
        MyDate due    = new MyDate(2024, 3, 1);
        MyDate ret    = new MyDate(2024, 4, 1);

        assertEquals(31L, library.daysOverdue(due, ret));
        assertEquals(15.50, library.computeFine(due, ret), EPS);
        assertTrue(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_returned_late_year_boundary() throws MyValueException {
        MyDate due    = new MyDate(2024, 12, 31);
        MyDate ret    = new MyDate(2025, 1, 5);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
    }

    @After
    public void tearDown() {
        library = null;
    }
}
