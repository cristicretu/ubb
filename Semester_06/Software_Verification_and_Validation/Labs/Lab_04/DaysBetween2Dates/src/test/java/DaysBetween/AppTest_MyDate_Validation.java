package DaysBetween;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Validation tests for MyDate's constructor.
 *
 * The previous version accepted impossible dates such as 2023-02-31
 * (which then led daysBetween2Dates to compute 3 days between
 * 2023-02-28 and 2023-02-31). The constructor now rejects such
 * inputs by checking the day against the actual length of the given
 * month, taking leap-year February into account.
 */
public class AppTest_MyDate_Validation {

    @Test(expected = MyValueException.class)
    public void test_feb_31_in_common_year_rejected() throws MyValueException {
        new MyDate(2023, 2, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_feb_31_in_leap_year_rejected() throws MyValueException {
        new MyDate(2024, 2, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_feb_29_in_common_year_rejected() throws MyValueException {
        new MyDate(2023, 2, 29);
    }

    @Test
    public void test_feb_29_in_leap_year_accepted() throws MyValueException {
        MyDate d = new MyDate(2024, 2, 29);
        assertEquals(29, d.getDay());
    }

    @Test
    public void test_feb_28_always_accepted() throws MyValueException {
        new MyDate(2023, 2, 28);
        new MyDate(2024, 2, 28);
        new MyDate(2100, 2, 28);
    }

    @Test(expected = MyValueException.class)
    public void test_april_31_rejected() throws MyValueException {
        new MyDate(2023, 4, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_june_31_rejected() throws MyValueException {
        new MyDate(2024, 6, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_september_31_rejected() throws MyValueException {
        new MyDate(2024, 9, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_november_31_rejected() throws MyValueException {
        new MyDate(2024, 11, 31);
    }

    @Test
    public void test_31_day_months_accepted() throws MyValueException {
        new MyDate(2024, 1, 31);
        new MyDate(2024, 3, 31);
        new MyDate(2024, 5, 31);
        new MyDate(2024, 7, 31);
        new MyDate(2024, 8, 31);
        new MyDate(2024, 10, 31);
        new MyDate(2024, 12, 31);
    }

    @Test(expected = MyValueException.class)
    public void test_day_zero_rejected() throws MyValueException {
        new MyDate(2024, 3, 0);
    }

    @Test(expected = MyValueException.class)
    public void test_negative_day_rejected() throws MyValueException {
        new MyDate(2024, 3, -1);
    }

    @Test(expected = MyValueException.class)
    public void test_month_zero_rejected() throws MyValueException {
        new MyDate(2024, 0, 15);
    }

    @Test(expected = MyValueException.class)
    public void test_month_13_rejected() throws MyValueException {
        new MyDate(2024, 13, 15);
    }

    @Test(expected = MyValueException.class)
    public void test_year_zero_rejected() throws MyValueException {
        new MyDate(0, 6, 15);
    }

    @Test(expected = MyValueException.class)
    public void test_negative_year_rejected() throws MyValueException {
        new MyDate(-1, 6, 15);
    }

    @Test
    public void test_centennial_non_leap_rejects_feb_29() {
        try {
            new MyDate(2100, 2, 29);
            org.junit.Assert.fail("2100 is not a leap year (divisible by 100, not 400)");
        } catch (MyValueException expected) {
            // good
        }
    }

    @Test
    public void test_quadricentennial_leap_accepts_feb_29() throws MyValueException {
        MyDate d = new MyDate(2000, 2, 29);
        assertEquals(29, d.getDay());
    }
}
