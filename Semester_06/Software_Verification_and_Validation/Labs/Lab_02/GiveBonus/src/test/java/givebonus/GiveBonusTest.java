package givebonus;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class GiveBonusTest {

    // ===== EQUIVALENCE PARTITIONING TEST CASES =====

    @Nested
    @DisplayName("EP - Valid Equivalence Classes")
    class EquivalencePartitioningValid {

        @Test
        @DisplayName("TC_EP_1: Regular employee, salary<=5000, not manager -> +1000, return 0")
        void tc_ep_1_regularEmployee_salaryBelow5000_bonus1000() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000)
            );

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_EP_2: Regular employee, salary>5000, not manager -> +500, return 0")
        void tc_ep_2_regularEmployee_salaryAbove5000_bonus500() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 6000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000)
            );

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(6500, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_EP_3: Manager, salary<=5000 -> +500, return 0")
        void tc_ep_3_manager_salaryBelow5000_bonus500() {
            List<Employee> employees = Arrays.asList(
                    new Employee("Jane", "IT", "manager", 4000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000)
            );

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(4500, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_EP_4: Manager, salary>5000 -> +500, return 0")
        void tc_ep_4_manager_salaryAbove5000_bonus500() {
            List<Employee> employees = Arrays.asList(
                    new Employee("Jane", "IT", "manager", 7000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000)
            );

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(7500, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_EP_5: Mixed employees in top dept - different bonuses applied correctly")
        void tc_ep_5_mixedEmployees_differentBonuses() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000),
                    new Employee("Jane", "IT", "manager", 4000),
                    new Employee("Bob", "IT", "developer", 6000),
                    new Employee("Alice", "HR", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000),
                    new Sale("HR", 30000)
            );

            int result = GiveBonus.giveBonus(4, employees, 2, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());  // John: +1000
            assertEquals(4500, employees.get(1).getSalary());  // Jane: manager -> +500
            assertEquals(6500, employees.get(2).getSalary());  // Bob: salary>5000 -> +500
            assertEquals(3000, employees.get(3).getSalary());  // Alice: HR, not top dept, unchanged
        }

        @Test
        @DisplayName("TC_EP_6: Multiple departments, only top dept employees get bonus")
        void tc_ep_6_multipleDepartments_onlyTopDeptGetBonus() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000),
                    new Employee("Bob", "HR", "developer", 4000),
                    new Employee("Eve", "Sales", "developer", 2000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000),
                    new Sale("HR", 30000),
                    new Sale("Sales", 10000)
            );

            int result = GiveBonus.giveBonus(3, employees, 3, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());  // IT top -> +1000
            assertEquals(4000, employees.get(1).getSalary());  // HR unchanged
            assertEquals(2000, employees.get(2).getSalary());  // Sales unchanged
        }
    }

    @Nested
    @DisplayName("EP - Invalid Equivalence Classes")
    class EquivalencePartitioningInvalid {

        @Test
        @DisplayName("TC_EP_7: numberOfEmployees=0 -> return 1")
        void tc_ep_7_zeroEmployees_return1() {
            List<Employee> employees = new ArrayList<>();
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(0, employees, 1, sales);

            assertEquals(1, result);
        }

        @Test
        @DisplayName("TC_EP_8: numberSales=0 -> return 1")
        void tc_ep_8_zeroSales_return1() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000)
            );
            List<Sale> sales = new ArrayList<>();

            int result = GiveBonus.giveBonus(1, employees, 0, sales);

            assertEquals(1, result);
        }

        @Test
        @DisplayName("TC_EP_9: Both numberOfEmployees=0 and numberSales=0 -> return 1")
        void tc_ep_9_bothZero_return1() {
            int result = GiveBonus.giveBonus(0, new ArrayList<>(), 0, new ArrayList<>());

            assertEquals(1, result);
        }

        @Test
        @DisplayName("TC_EP_10: No employees in dept with biggest sale -> return 2")
        void tc_ep_10_noEmployeesInTopDept_return2() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "HR", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000),
                    new Sale("HR", 30000)
            );

            int result = GiveBonus.giveBonus(1, employees, 2, sales);

            assertEquals(2, result);
        }
    }

    // ===== BOUNDARY VALUE ANALYSIS TEST CASES =====

    @Nested
    @DisplayName("BVA - numberOfEmployees boundaries")
    class BVA_NumberOfEmployees {

        @Test
        @DisplayName("TC_BVA_1: numberOfEmployees=0 (boundary) -> return 1")
        void tc_bva_1_numEmployees_0() {
            int result = GiveBonus.giveBonus(0, new ArrayList<>(), 1,
                    Arrays.asList(new Sale("IT", 50000)));
            assertEquals(1, result);
        }

        @Test
        @DisplayName("TC_BVA_2: numberOfEmployees=1, single employee in top dept -> return 0")
        void tc_bva_2_numEmployees_1() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());
        }
    }

    @Nested
    @DisplayName("BVA - numberSales boundaries")
    class BVA_NumberOfSales {

        @Test
        @DisplayName("TC_BVA_3: numberSales=0 (boundary) -> return 1")
        void tc_bva_3_numSales_0() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000)
            );

            int result = GiveBonus.giveBonus(1, employees, 0, new ArrayList<>());

            assertEquals(1, result);
        }

        @Test
        @DisplayName("TC_BVA_4: numberSales=1, single sale -> return 0")
        void tc_bva_4_numSales_1() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());
        }
    }

    @Nested
    @DisplayName("BVA - Salary boundaries (around 5000)")
    class BVA_Salary {

        @Test
        @DisplayName("TC_BVA_5: salary=4999 (just below 5000), not manager -> +1000")
        void tc_bva_5_salary_4999_notManager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 4999)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(5999, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_6: salary=5000 (exact boundary), not manager -> +1000")
        void tc_bva_6_salary_5000_notManager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 5000)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(6000, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_7: salary=5001 (just above 5000), not manager -> +500")
        void tc_bva_7_salary_5001_notManager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 5001)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(5501, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_8: salary=4999, manager -> +500 (manager overrides)")
        void tc_bva_8_salary_4999_manager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("Jane", "IT", "manager", 4999)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(5499, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_9: salary=5000, manager -> +500")
        void tc_bva_9_salary_5000_manager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("Jane", "IT", "manager", 5000)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(5500, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_10: salary=5001, manager -> +500")
        void tc_bva_10_salary_5001_manager() {
            List<Employee> employees = Arrays.asList(
                    new Employee("Jane", "IT", "manager", 5001)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(5501, employees.get(0).getSalary());
        }
    }

    @Nested
    @DisplayName("BVA - Edge cases")
    class BVA_EdgeCases {

        @Test
        @DisplayName("TC_BVA_11: All employees in top dept get bonus")
        void tc_bva_11_allEmployeesInTopDept() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000),
                    new Employee("Jane", "IT", "developer", 4000)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(2, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(4000, employees.get(0).getSalary());
            assertEquals(5000, employees.get(1).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_12: No employees in top dept, all in other dept -> return 2")
        void tc_bva_12_noEmployeesInTopDept() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "HR", "developer", 3000),
                    new Employee("Jane", "HR", "developer", 4000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("IT", 50000),
                    new Sale("HR", 30000)
            );

            int result = GiveBonus.giveBonus(2, employees, 2, sales);

            assertEquals(2, result);
            assertEquals(3000, employees.get(0).getSalary());
            assertEquals(4000, employees.get(1).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_13: salary=1 (minimum valid), not manager -> +1000")
        void tc_bva_13_salary_minimum() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 1)
            );
            List<Sale> sales = Arrays.asList(new Sale("IT", 50000));

            int result = GiveBonus.giveBonus(1, employees, 1, sales);

            assertEquals(0, result);
            assertEquals(1001, employees.get(0).getSalary());
        }

        @Test
        @DisplayName("TC_BVA_14: Two departments with different sales, correct top dept identified")
        void tc_bva_14_correctTopDeptIdentified() {
            List<Employee> employees = Arrays.asList(
                    new Employee("John", "IT", "developer", 3000),
                    new Employee("Bob", "HR", "developer", 3000)
            );
            List<Sale> sales = Arrays.asList(
                    new Sale("HR", 60000),
                    new Sale("IT", 50000)
            );

            int result = GiveBonus.giveBonus(2, employees, 2, sales);

            assertEquals(0, result);
            assertEquals(3000, employees.get(0).getSalary());  // IT unchanged
            assertEquals(4000, employees.get(1).getSalary());  // HR top -> +1000
        }
    }
}
