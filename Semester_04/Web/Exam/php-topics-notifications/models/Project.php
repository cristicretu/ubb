<?php
require_once 'config/database.php';

class Project {
    public $id;
    public $projectManagerID;
    public $name;
    public $description;
    public $members;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
        $this->initializeTable();
    }

    private function initializeTable() {
        $query = "CREATE TABLE IF NOT EXISTS Project (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ProjectManagerID INTEGER,
            name VARCHAR(255),
            description TEXT,
            members TEXT
        )";
        $this->db->exec($query);
    }

    public function getAllProjects() {
        $query = "SELECT id, ProjectManagerID, name, description, members FROM Project";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function getProjectsByManagerId($projectManagerId) {
        $query = "SELECT id, ProjectManagerID, name, description, members FROM Project WHERE ProjectManagerID = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$projectManagerId]);
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function createProject($name, $projectManagerId, $description, $members) {
        $query = "INSERT INTO Project (name, ProjectManagerID, description, members) VALUES (?, ?, ?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$name, $projectManagerId, $description, $members]);
    }

    public function updateProject($id, $projectManagerId, $name, $description, $members) {
        $query = "UPDATE Project SET ProjectManagerID = ?, name = ?, description = ?, members = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$projectManagerId, $name, $description, $members, $id]);
    }

    public function deleteProject($id) {
        $query = "DELETE FROM Project WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }

    public function getProjectByName($name) {
        $query = "SELECT id, ProjectManagerID, name, description, members FROM Project WHERE name = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$name]);
        return $stmt->fetch(PDO::FETCH_ASSOC);
    }

    public function assignProject($projectName, $projectManagerId) {
        // Check if project exists
        $existingProject = $this->getProjectByName($projectName);
        if ($existingProject) {
            return $this->updateProject($existingProject['id'], $projectManagerId, $projectName, 
                                      $existingProject['description'], $existingProject['members']);
        }
        return false;
    }
}
?> 