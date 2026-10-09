package ssvv;

public class GiveNextDate {

    private int month, day, year;

    public GiveNextDate(int m, int d, int y) {
        month = m;
        day = d;
        year = y;
    }

    public String run() {
        if (day < 1 || month < 1 || month > 12 || year < 1801 || year > 2021)
            return "invalid Input Date";

        int tomorrowDay = day;
        int tomorrowMonth = month;
        int tomorrowYear = year;

        if (isThirtyOneDayMonth(month)) {
            if (day < 31)
                tomorrowDay = day + 1;
            else {
                tomorrowDay = 1;
                tomorrowMonth = month + 1;
            }
        } else if (isThirtyDayMonth(month)) {
            if (day < 30)
                tomorrowDay = day + 1;
            else {
                if (day == 30) {
                    tomorrowDay = 1;
                    tomorrowMonth = month + 1;
                } else
                    return "Invalid Input Date";
            }
        } else if (isDecember(month)) {
            if (day < 31)
                tomorrowDay = day + 1;
            else {
                tomorrowDay = 1;
                tomorrowMonth = 1;
                if (year == 2021)
                    return "Invalid Next Year";
                else
                    tomorrowYear = year + 1;
            }
        } else if (isFebruary(month)) {
            if (day < 28)
                tomorrowDay = day + 1;
            else {
                if (day == 28) {
                    if (isLeapYear(year))
                        tomorrowDay = 29;
                    else {
                        tomorrowDay = 1;
                        tomorrowMonth = 3;
                    }
                } else if (day == 29) {
                    if (isLeapYear(year)) {
                        tomorrowDay = 1;
                        tomorrowMonth = 3;
                    } else
                        return "Invalid Input Date";
                } else if (day > 29)
                    return "Invalid Input Date";
            }
        }
        return tomorrowMonth + "/" + tomorrowDay + "/" + tomorrowYear;
    }

    private static boolean isThirtyOneDayMonth(int month) {
        return month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10;
    }

    private static boolean isThirtyDayMonth(int month) {
        return month == 4 || month == 6 || month == 9 || month == 11;
    }

    private static boolean isDecember(int month) {
        return month == 12;
    }

    private static boolean isFebruary(int month) {
        return month == 2;
    }

    private static boolean isLeapYear(int year) {
        if ((year % 100) == 0)
            return (year % 400) == 0;
        else
            return (year % 4) == 0;
    }
}
