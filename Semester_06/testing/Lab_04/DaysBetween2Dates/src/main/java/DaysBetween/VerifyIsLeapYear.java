package DaysBetween;

/**
 * Dependency that exposes the leap-year predicate, intentionally
 * not yet implemented (mirrors VerifyIsPrime in the StubsIsPrime
 * example). It is meant to be mocked with Mockito by the integration
 * tests, so the body always returns true and is never actually
 * exercised in production.
 */
public class VerifyIsLeapYear {

    public VerifyIsLeapYear() {
        System.out.println("VerifyIsLeapYear (dependency, to be mocked) ...");
    }

    public boolean isLeapYear(int year) throws MyValueException {
        /*
        // Real implementation lives in DaysBetween#isLeapYear; this
        // class is deliberately a hollow seam so tests can mock it.
        if (year < 1) {
            throw new MyValueException("year must be >= 1");
        }
        if ((year % 400) == 0) return true;
        if ((year % 100) == 0) return false;
        return (year % 4) == 0;
        */
        return true;
    }
}
