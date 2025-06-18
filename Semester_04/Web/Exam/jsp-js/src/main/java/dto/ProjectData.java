package dto;

public class ProjectData {
  public int id;
  public int projectManagerID;
  public String name;
  public String description;
  public String members;

  public ProjectData() {
  }

  public ProjectData(int id, int projectManagerID, String name, String description, String members) {
    this.id = id;
    this.projectManagerID = projectManagerID;
    this.name = name;
    this.description = description;
    this.members = members;
  }
}