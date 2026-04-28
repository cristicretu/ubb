package DaysBetween;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

/**
 * Regression tests for the "general implementation" requirement (1).
 *
 * The default DaysBetween must work for ARBITRARY years (not just
 * the years that happen to live in isLeapYearStub). The previous
 * version of daysBetween2Dates was wired to the stub and threw
 * MyValueException for years outside the stub table (e.g. 2027).
 */
public class AppTest_DaysBetween2Dates_General {

    private DaysBetween obj;

    @Before
    public void setup() {
        obj = new DaysBetween();
    }

    /**
     * The exact failure reported by the reviewer:
     * 2027-02-28 -> 2027-03-01 must succeed and return 1
     * (2027 is a common year, so February has 28 days).
     */
    @Test
    public void test_year_outside_stub_table_2027_common() throws MyValueException {
        MyDate d1 = new MyDate(2027, 2, 28);
        MyDate d2 = new MyDate(2027, 3, 1);
        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_year_outside_stub_table_2028_leap() throws MyValueException {
        MyDate d1 = new MyDate(2028, 2, 28);
        MyDate d2 = new MyDate(2028, 3, 1);
        assertEquals(2L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_far_future_year_works() throws MyValueException {
        MyDate d1 = new MyDate(2099, 12, 31);
        MyDate d2 = new MyDate(2100, 1, 1);
        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_centennial_non_leap_correct_feb() throws MyValueException {
        MyDate d1 = new MyDate(2100, 2, 28);
        MyDate d2 = new MyDate(2100, 3, 1);
        assertEquals(1L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_quadricentennial_leap_correct_feb() throws MyValueException {
        MyDate d1 = new MyDate(2000, 2, 28);
        MyDate d2 = new MyDate(2000, 3, 1);
        assertEquals(2L, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_multi_decade_span() throws MyValueException {
        MyDate d1 = new MyDate(1995, 6, 15);
        MyDate d2 = new MyDate(2030, 6, 15);
        long expected = 0;
        for (int y = 1995; y < 2030; y++) {
            expected += obj.isLeapYear(y) ? 366 : 365;
        }
        assertEquals(expected, obj.daysBetween2Dates(d1, d2));
    }

    @Test
    public void test_stub_mode_still_throws_for_unknown_years() throws MyValueException {
        DaysBetween stubMode = new DaysBetween(true);
        try {
            stubMode.daysBetween2Dates(
                    new MyDate(2027, 2, 28),
                    new MyDate(2027, 3, 1));
            fail("stub mode should throw for years not in the stub table");
        } catch (MyValueException expected) {
            // exactly what we want: stub mode is honest about its scope
        }
    }

    @After
    public void tearDown() {
        obj = null;
    }
}
