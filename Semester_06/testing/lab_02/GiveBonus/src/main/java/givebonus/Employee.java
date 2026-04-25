package givebonus;

public class Employee {
    private String name;
    private String department;
    private String function;
    private int salary;

    public Employee(String name, String department, String function, int salary) {
        this.name = name;
        this.department = department;
        this.function = function;
        this.salary = salary;
    }

    public String getName() { return name; }
    public String getDepartment() { return department; }
    public String getFunction() { return function; }
    public int getSalary() { return salary; }

    public void setSalary(int salary) { this.salary = salary; }

    @Override
    public String toString() {
        return name + " " + department + " " + function + " " + salary;
    }
}
