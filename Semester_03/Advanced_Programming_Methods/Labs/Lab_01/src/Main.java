// Main class
public class Main {
    public static void main(String[] args) {
        // Creating a subclass object
        Dog dog = new Dog("Buddy", 3);
        dog.speak(); // Calls overridden method
        
        // Accessing static method and field
        System.out.println("Number of animals: " + Animal.getCount());
        
        // Using an exception
        try {
            dog.checkAge(15); // Valid age
            dog.checkAge(150); // Invalid age, will throw exception
        } catch (IllegalArgumentException e) {
            System.out.println(e.getMessage());
        }
    }
}

// Base class
class Animal {
    String name;
    int age;
    
    // Static field
    public static int animalCount = 0;
    
    // Constructor
    public Animal(String name, int age) {
        this.name = name;
        this.age = age;
        animalCount++;
    }
    
    // Static method to get animal count
    public static int getCount() {
        return animalCount;
    }
    
    // Method to be overridden
    public void speak() {
        System.out.println("The animal makes a sound.");
    }
    
    // Method to check age (throws exception)
    public void checkAge(int age) {
        if (age > 100) {
            throw new IllegalArgumentException("Age can't be greater than 100!");
        }
        System.out.println("Age is valid.");
    }
}

// Derived subclass
class Dog extends Animal {
    // Constructor for Dog
    public Dog(String name, int age) {
        super(name, age);
    }
    
    // Overriding the speak method
    @Override
    public void speak() {
        System.out.println(name + " barks!");
    }
}
