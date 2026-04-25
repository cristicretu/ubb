package DaysBetween;

public class App {
    public static void main(String[] args) throws MyValueException {
        DaysBetween db = new DaysBetween();
        MyDate due    = new MyDate(2024, 2, 15);
        MyDate ret    = new MyDate(2024, 3, 10);
        long overdue  = db.daysBetween2Dates(due, ret);

        LibraryReturnSystem lib = new LibraryReturnSystem(db);
        System.out.println("Days between "
                + due + " and " + ret + " = " + overdue);
        System.out.println("Library fine = " + lib.computeFine(due, ret));
    }
}
