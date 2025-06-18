package dto;

public class DeveloperData {
  public int id;
  public String name;
  public int age;
  public String skills;

  public DeveloperData() {
  }

  public DeveloperData(int id, String name, int age, String skills) {
    this.id = id;
    this.name = name;
    this.age = age;
    this.skills = skills;
  }
}