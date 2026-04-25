package DaysBetween;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.lenient;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

/**
 * Mockito-based integration tests for daysBetween2Dates.
 *
 * Mirrors AppTest_LongestSeq_IsPrimeMock: the leap-year predicate is
 * not provided by a stub method but by a Mockito mock of
 * VerifyIsLeapYear, injected into DaysBetween_VerifyIsLeapYearMock.
 */
public class AppTest_DaysBetween2Dates_IsLeapYearMock {

    @Mock
    private VerifyIsLeapYear verifyIsLeapYear;

    private DaysBetween_VerifyIsLeapYearMock obj;

    @Before
    public void setup() {
        verifyIsLeapYear = mock(VerifyIsLeapYear.class);
        obj = new DaysBetween_VerifyIsLeapYearMock(verifyIsLeapYear);
    }

    @Test
    public void test_leap_year_feb_to_march() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate d1 = new MyDate(2024, 2, 28);
        MyDate d2 = new MyDate(2024, 3, 1);

        assertEquals(2L, obj.daysBetween2Dates(d1, d2));
        verify(verifyIsLeapYear, atLeastOnce()).isLeapYear(2024);
    }

    @Test
    public void test_common_year_feb_to_march() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2023)).thenReturn(false);

        MyDate d1 = new MyDate(2023, 2, 28);
        MyDate d2 = new MyDate(2023, 3, 1);

        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
        verify(verifyIsLeapYear, atLeastOnce()).isLeapYear(2023);
    }

    @Test
    public void test_year_boundary_dec_to_jan() throws MyValueException {
        lenient().when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);
        lenient().when(verifyIsLeapYear.isLeapYear(2025)).thenReturn(false);

        MyDate d1 = new MyDate(2024, 12, 31);
        MyDate d2 = new MyDate(2025, 1, 1);

        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_multi_year_with_leap() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2020)).thenReturn(true);
        when(verifyIsLeapYear.isLeapYear(2021)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2022)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2023)).thenReturn(false);
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate d1 = new MyDate(2020, 2, 15);
        MyDate d2 = new MyDate(2024, 3, 10);

        assertEquals(1485L, obj.daysBetween2Dates(d1, d2));
        verify(verifyIsLeapYear).isLeapYear(2020);
        verify(verifyIsLeapYear).isLeapYear(2021);
        verify(verifyIsLeapYear).isLeapYear(2022);
        verify(verifyIsLeapYear).isLeapYear(2023);
        verify(verifyIsLeapYear, atLeastOnce()).isLeapYear(2024);
    }

    @Test
    public void test_reversed_arguments_uses_mock_for_correct_year() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(anyInt())).thenReturn(false);

        MyDate d1 = new MyDate(2023, 3, 15);
        MyDate d2 = new MyDate(2023, 3, 10);

        assertEquals(5L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_full_leap_year_uses_mock_to_decide_366() throws MyValueException {
        when(verifyIsLeapYear.isLeapYear(2024)).thenReturn(true);

        MyDate d1 = new MyDate(2024, 1, 1);
        MyDate d2 = new MyDate(2024, 12, 31);

        assertEquals(365L, obj.daysBetween2Dates(d1, d2));
    }
}
