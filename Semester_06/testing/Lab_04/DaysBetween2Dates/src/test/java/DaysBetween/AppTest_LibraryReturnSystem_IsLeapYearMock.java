package DaysBetween;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Application-context integration test, Mockito flavour.
 *
 * The system under test is wired against
 * DaysBetween_VerifyIsLeapYearMock, which itself delegates the
 * leap-year predicate to a mocked VerifyIsLeapYear. The chain is:
 *
 *     LibraryReturnSystem_Mock
 *         -> DaysBetween_VerifyIsLeapYearMock.daysBetween2Dates
 *             -> VerifyIsLeapYear.isLeapYear      (mocked)
 */
public class AppTest_LibraryReturnSystem_IsLeapYearMock {

    private static final double EPS = 1e-9;

    @Mock
    private VerifyIsLeapYear verifyIsLeapYear;

    private DaysBetween_VerifyIsLeapYearMock daysBetween;
    private LibraryReturnSystem_Mock library;

    @Before
    public void setup() {
        verifyIsLeapYear = mock(VerifyIsLeapYear.class);
        daysBetween      = new DaysBetween_VerifyIsLeapYearMock(verifyIsLeapYear);
        library          = new LibraryReturnSystem_Mock(daysBetween, 0.50);
    }

    @Test
    public void test_returned_on_time_no_fine() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(anyInt())).thenReturn(false);

        MyDate due = new MyDate(2025, 4, 10);
        MyDate ret = new MyDate(2025, 4, 10);

        assertEquals(0L, library.daysOverdue(due, ret));
        assertEquals(0.0, library.computeFine(due, ret), EPS);
        assertFalse(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_returned_early_no_fine() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(anyInt())).thenReturn(false);

        MyDate due = new MyDate(2025, 4, 15);
        MyDate ret = new MyDate(2025, 4, 10);

        assertEquals(0L, library.daysOverdue(due, ret));
        assertEquals(0.0, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_5_days_late_in_common_year() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2025)).thenReturn(false);

        MyDate due = new MyDate(2025, 4, 10);
        MyDate ret = new MyDate(2025, 4, 15);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
        verify(verifyIsLeapYear, atLeastOnce()).isLeapYear(2025);
    }

    @Test
    public void test_late_return_across_leap_february() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate due = new MyDate(2024, 2, 28);
        MyDate ret = new MyDate(2024, 3, 5);

        assertEquals(6L, library.daysOverdue(due, ret));
        assertEquals(3.00, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_late_return_across_common_february() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2023)).thenReturn(false);

        MyDate due = new MyDate(2023, 2, 28);
        MyDate ret = new MyDate(2023, 3, 5);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_31_days_late_revoke_privileges() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate due = new MyDate(2024, 3, 1);
        MyDate ret = new MyDate(2024, 4, 1);

        assertEquals(31L, library.daysOverdue(due, ret));
        assertEquals(15.50, library.computeFine(due, ret), EPS);
        assertTrue(library.shouldRevokePrivileges(due, ret));
    }

    @Test
    public void test_late_return_year_boundary() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);
        when(verifyIsLeapYear.isLeapYear(2025)).thenReturn(false);

        MyDate due = new MyDate(2024, 12, 31);
        MyDate ret = new MyDate(2025, 1, 5);

        assertEquals(5L, library.daysOverdue(due, ret));
        assertEquals(2.50, library.computeFine(due, ret), EPS);
    }

    @Test
    public void test_long_overdue_multi_year() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2020)).thenReturn(true);
        when(verifyIsLeapYear.isLeapYear(2021)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2022)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2023)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate due = new MyDate(2020, 2, 15);
        MyDate ret = new MyDate(2024, 3, 10);

        assertEquals(1485L, library.daysOverdue(due, ret));
        assertTrue(library.shouldRevokePrivileges(due, ret));
        assertEquals(742.50, library.computeFine(due, ret), EPS);
    }
}
