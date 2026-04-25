package givebonus;

import java.io.*;
import java.util.*;

public class GiveBonus {

    /**
     * Increases the salary of employees whose department had the biggest sale.
     * - Regular employees with salary <= 5000: +1000
     * - Employees with salary > 5000 OR managers: +500
     *
     * @return 0 if computed correctly,
     *         1 if numberOfEmployees or numberSales is 0,
     *         2 if no employees in the department with the biggest sale
     */
    public static int giveBonus(int numberOfEmployees, List<Employee> employees,
                                int numberSales, List<Sale> sales) {
        if (numberOfEmployees == 0 || numberSales == 0) {
            return 1;
        }

        String topDepartment = null;
        int maxSale = Integer.MIN_VALUE;
        for (Sale sale : sales) {
            if (sale.getSumSale() > maxSale) {
                maxSale = sale.getSumSale();
                topDepartment = sale.getDepartment();
            }
        }

        boolean foundEmployee = false;
        for (Employee emp : employees) {
            if (emp.getDepartment().equals(topDepartment)) {
                foundEmployee = true;
                if (emp.getSalary() > 5000 || emp.getFunction().equalsIgnoreCase("manager")) {
                    emp.setSalary(emp.getSalary() + 500);
                } else {
                    emp.setSalary(emp.getSalary() + 1000);
                }
            }
        }

        if (!foundEmployee) {
            return 2;
        }

        return 0;
    }

    public static void main(String[] args) {
        try {
            Scanner scanner = new Scanner(new File("IN.TXT"));

            int numberOfEmployees = Integer.parseInt(scanner.nextLine().trim());
            List<Employee> employees = new ArrayList<>();
            for (int i = 0; i < numberOfEmployees; i++) {
                String line = scanner.nextLine().trim();
                String[] parts = line.split("\\s+");
                String name = parts[0];
                String department = parts[1];
                String function = parts[2];
                int salary = Integer.parseInt(parts[3]);
                employees.add(new Employee(name, department, function, salary));
            }

            int numberSales = Integer.parseInt(scanner.nextLine().trim());
            List<Sale> sales = new ArrayList<>();
            for (int i = 0; i < numberSales; i++) {
                String line = scanner.nextLine().trim();
                String[] parts = line.split("\\s+");
                String department = parts[0];
                int sumSale = Integer.parseInt(parts[1]);
                sales.add(new Sale(department, sumSale));
            }
            scanner.close();

            int result = giveBonus(numberOfEmployees, employees, numberSales, sales);

            PrintWriter writer = new PrintWriter(new FileWriter("OUT.TXT"));
            writer.println(result);
            for (Employee emp : employees) {
                writer.println(emp.toString());
            }
            writer.close();

        } catch (Exception e) {
            try {
                PrintWriter writer = new PrintWriter(new FileWriter("OUT.TXT"));
                writer.println("Error: " + e.getMessage());
                writer.close();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }
    }
}
