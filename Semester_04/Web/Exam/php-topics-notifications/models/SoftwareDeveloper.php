<?php
require_once 'config/database.php';

class SoftwareDeveloper {
    public $id;
    public $name;
    public $age;
    public $skills;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
        $this->initializeTable();
    }

    private function initializeTable() {
        $query = "CREATE TABLE IF NOT EXISTS SoftwareDeveloper (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name VARCHAR(255),
            age INTEGER,
            skills TEXT
        )";
        $this->db->exec($query);
    }

    public function getAllDevelopers() {
        $query = "SELECT id, name, age, skills FROM SoftwareDeveloper";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function getDeveloperByName($name) {
        $query = "SELECT id, name, age, skills FROM SoftwareDeveloper WHERE name = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$name]);
        return $stmt->fetch(PDO::FETCH_ASSOC);
    }

    public function createDeveloper($name, $age, $skills) {
        // Clean and sanitize input
        $cleanName = trim(strip_tags($name));
        $cleanAge = (int) trim(strip_tags($age));
        $cleanSkills = trim(strip_tags($skills));

        $query = "INSERT INTO SoftwareDeveloper (name, age, skills) VALUES (?, ?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$cleanName, $cleanAge, $cleanSkills]);
    }

    public function updateDeveloper($id, $name, $age, $skills) {
        // Clean and sanitize input
        $cleanName = trim(strip_tags($name));
        $cleanAge = (int) trim(strip_tags($age));
        $cleanSkills = trim(strip_tags($skills));

        $query = "UPDATE SoftwareDeveloper SET name = ?, age = ?, skills = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$cleanName, $cleanAge, $cleanSkills, $id]);
    }

    public function deleteDeveloper($id) {
        $query = "DELETE FROM SoftwareDeveloper WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }
}
?> 