package ssvv;

import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class GiveNextDateTest {

    private String nextDate(int month, int day, int year) {
        return new GiveNextDate(month, day, year).run();
    }

    // --- Validation (Path 1): input boundary checks ---

    @Test
    public void tc01_invalidMonthLow() {
        assertEquals("invalid Input Date", nextDate(0, 15, 1900));
    }

    @Test
    public void tc02_invalidMonthHigh() {
        assertEquals("invalid Input Date", nextDate(13, 1, 1900));
    }

    @Test
    public void tc03_invalidDayLow() {
        assertEquals("invalid Input Date", nextDate(1, 0, 1900));
    }

    @Test
    public void tc04_invalidYearLow() {
        assertEquals("invalid Input Date", nextDate(1, 15, 1800));
    }

    @Test
    public void tc05_invalidYearHigh() {
        assertEquals("invalid Input Date", nextDate(1, 15, 2022));
    }

    // --- 31-day month paths ---

    @Test
    public void tc06_thirtyOneDayMonth_dayLessThan31() {
        assertEquals("1/16/1900", nextDate(1, 15, 1900));
    }

    @Test
    public void tc07_thirtyOneDayMonth_day31() {
        assertEquals("2/1/1900", nextDate(1, 31, 1900));
    }

    @Test
    public void tc08_july_dayIncrement() {
        assertEquals("7/21/1900", nextDate(7, 20, 1900));
    }

    // --- 30-day month paths ---

    @Test
    public void tc09_thirtyDayMonth_dayLessThan30() {
        assertEquals("4/16/1900", nextDate(4, 15, 1900));
    }

    @Test
    public void tc10_thirtyDayMonth_day30() {
        assertEquals("5/1/1900", nextDate(4, 30, 1900));
    }

    @Test
    public void tc11_thirtyDayMonth_dayOver30() {
        assertEquals("Invalid Input Date", nextDate(4, 31, 1900));
    }

    // --- December paths ---

    @Test
    public void tc12_december_dayLessThan31() {
        assertEquals("12/16/1900", nextDate(12, 15, 1900));
    }

    @Test
    public void tc13_december31_yearRollover() {
        assertEquals("1/1/1901", nextDate(12, 31, 1900));
    }

    @Test
    public void tc14_december31_maxYear() {
        assertEquals("Invalid Next Year", nextDate(12, 31, 2021));
    }

    // --- February paths ---

    @Test
    public void tc15_february_dayLessThan28() {
        assertEquals("2/16/1900", nextDate(2, 15, 1900));
    }

    @Test
    public void tc16_february28_leapYear() {
        assertEquals("2/29/2000", nextDate(2, 28, 2000));
    }

    @Test
    public void tc17_february28_nonLeapYear() {
        assertEquals("3/1/1900", nextDate(2, 28, 1900));
    }

    @Test
    public void tc18_february29_leapYear() {
        assertEquals("3/1/2000", nextDate(2, 29, 2000));
    }

    @Test
    public void tc19_february29_nonLeapYear() {
        assertEquals("Invalid Input Date", nextDate(2, 29, 1900));
    }

    @Test
    public void tc20_february_dayOver29() {
        assertEquals("Invalid Input Date", nextDate(2, 30, 1900));
    }
}
