package DaysBetween;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Driver tests for daysBetween2Dates, exercised against
 * DaysBetween.isLeapYearStub (the hard-coded leap-year stub).
 *
 * Mirrors AppTest_LongestSeq_IsPrimeStub from the example: the
 * driver invokes the production method but the leap-year predicate
 * underneath is the stub, so the test focuses on the integration
 * logic of daysBetween2Dates rather than on isLeapYear itself.
 */
public class AppTest_DaysBetween2Dates_IsLeapYearStub {

    private DaysBetween obj;

    @Before
    public void setup() {
        obj = new DaysBetween();
    }

    @Test
    public void test_TC_1_same_date_zero_days() throws MyValueException {
        MyDate d = new MyDate(2024, 3, 10);
        assertEquals(0L, obj.daysBetween2Dates(d, d));
    }

    @Test
    public void test_TC_2_same_year_no_feb() throws MyValueException {
        MyDate d1 = new MyDate(2024, 3, 10);
        MyDate d2 = new MyDate(2024, 3, 15);
        assertEquals(5L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_3_leap_year_feb_to_march() throws MyValueException {
        MyDate d1 = new MyDate(2024, 2, 28);
        MyDate d2 = new MyDate(2024, 3, 1);
        assertEquals(2L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_4_common_year_feb_to_march() throws MyValueException {
        MyDate d1 = new MyDate(2023, 2, 28);
        MyDate d2 = new MyDate(2023, 3, 1);
        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_5_year_boundary_dec_to_jan() throws MyValueException {
        MyDate d1 = new MyDate(2024, 12, 31);
        MyDate d2 = new MyDate(2025, 1, 1);
        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_6_multi_year_with_leap() throws MyValueException {
        MyDate d1 = new MyDate(2020, 2, 15);
        MyDate d2 = new MyDate(2024, 3, 10);
        assertEquals(1485L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_7_reversed_arguments_returns_absolute() throws MyValueException {
        MyDate d1 = new MyDate(2024, 3, 15);
        MyDate d2 = new MyDate(2024, 3, 10);
        assertEquals(5L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_8_full_leap_year_jan_to_dec() throws MyValueException {
        MyDate d1 = new MyDate(2024, 1, 1);
        MyDate d2 = new MyDate(2024, 12, 31);
        assertEquals(365L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_TC_9_full_common_year_jan_to_dec() throws MyValueException {
        MyDate d1 = new MyDate(2023, 1, 1);
        MyDate d2 = new MyDate(2023, 12, 31);
        assertEquals(364L, obj.daysBetween2Dates(d1, d2));
    }

    @After
    public void tearDown() {
        obj = null;
    }
}
