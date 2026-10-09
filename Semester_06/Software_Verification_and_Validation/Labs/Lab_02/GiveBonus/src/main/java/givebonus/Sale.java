package givebonus;

public class Sale {
    private String department;
    private int sumSale;

    public Sale(String department, int sumSale) {
        this.department = department;
        this.sumSale = sumSale;
    }

    public String getDepartment() { return department; }
    public int getSumSale() { return sumSale; }

    @Override
    public String toString() {
        return department + " " + sumSale;
    }
}
