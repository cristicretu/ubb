<?php
require_once 'config/database.php';

class Authors {
    public $id;
    public $name;
    public $email;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
        $this->initializeTable();
    }

    private function initializeTable() {
        $query = "CREATE TABLE IF NOT EXISTS Authors (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name VARCHAR(255),
            email VARCHAR(255)
        )";
        $this->db->exec($query);
    }

    public function getAllAuthors() {
        $query = "SELECT id, name, email FROM Authors";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function createAuthor($name, $email) {
        $query = "INSERT INTO Authors (name, email) VALUES (?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$name, $email]);
    }

    public function updateAuthor($id, $name, $email) {
        $query = "UPDATE Authors SET name = ?, email = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$name, $email, $id]);
    }

    public function deleteAuthor($id) {
        $query = "DELETE FROM Authors WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }
}
?> 