package DaysBetween;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Equivalence class + boundary value tests for the real isLeapYear
 * implementation. Mirrors the role of AppTest_IsPrime_BBT in the
 * StubsIsPrime example.
 *
 * Equivalence classes for isLeapYear(year):
 *   EC1: year >= 1 and divisible by 400                 -> leap
 *   EC2: year >= 1 divisible by 100 but not 400         -> common
 *   EC3: year >= 1 divisible by 4 but not by 100        -> leap
 *   EC4: year >= 1 not divisible by 4                   -> common
 *   EC5: year <  1                                      -> MyValueException
 */
public class AppTest_IsLeapYear_BBT {

    private DaysBetween obj;

    @Before
    public void setup() {
        obj = new DaysBetween();
    }

    @Test
    public void test_TC_1_EC_div_by_400_isLeap() throws MyValueException {
        assertTrue(obj.isLeapYear(2000));
    }

    @Test
    public void test_TC_2_EC_div_by_100_not_400_isCommon() throws MyValueException {
        assertFalse(obj.isLeapYear(1900));
    }

    @Test
    public void test_TC_3_EC_div_by_4_not_100_isLeap() throws MyValueException {
        assertTrue(obj.isLeapYear(2024));
    }

    @Test
    public void test_TC_4_EC_not_div_by_4_isCommon() throws MyValueException {
        assertFalse(obj.isLeapYear(2023));
    }

    @Test(expected = MyValueException.class)
    public void test_TC_5_EC_invalid_year_throws() throws MyValueException {
        obj.isLeapYear(0);
    }

    @Test
    public void test_TC_1_BVA_year_1_isCommon() throws MyValueException {
        assertFalse(obj.isLeapYear(1));
    }

    @Test
    public void test_TC_2_BVA_year_4_isLeap() throws MyValueException {
        assertTrue(obj.isLeapYear(4));
    }

    @Test
    public void test_TC_3_BVA_year_100_isCommon() throws MyValueException {
        assertFalse(obj.isLeapYear(100));
    }

    @Test
    public void test_TC_4_BVA_year_400_isLeap() throws MyValueException {
        assertTrue(obj.isLeapYear(400));
    }

    @Test(expected = MyValueException.class)
    public void test_TC_5_BVA_year_minus_1_throws() throws MyValueException {
        obj.isLeapYear(-1);
    }

    @Test
    public void test_TC_6_BVA_year_INT_MAX() throws MyValueException {
        assertEquals((Integer.MAX_VALUE % 4 == 0)
                        && ((Integer.MAX_VALUE % 100 != 0)
                            || (Integer.MAX_VALUE % 400 == 0)),
                obj.isLeapYear(Integer.MAX_VALUE));
    }

    @After
    public void tearDown() {
        obj = null;
    }
}
