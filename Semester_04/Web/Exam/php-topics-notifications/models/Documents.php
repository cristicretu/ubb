<?php
require_once 'config/database.php';

class Documents {
    public $id;
    public $title;
    public $content;
    public $authorId;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
        $this->initializeTable();
    }

    private function initializeTable() {
        $query = "CREATE TABLE IF NOT EXISTS Documents (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            title VARCHAR(255),
            content TEXT,
            authorId INTEGER
        )";
        $this->db->exec($query);
    }

    public function getAllDocuments() {
        $query = "SELECT id, title, content, authorId FROM Documents";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function createDocument($title, $content, $authorId) {
        $query = "INSERT INTO Documents (title, content, authorId) VALUES (?, ?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$title, $content, $authorId]);
    }

    public function updateDocument($id, $title, $content, $authorId) {
        $query = "UPDATE Documents SET title = ?, content = ?, authorId = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$title, $content, $authorId, $id]);
    }

    public function deleteDocument($id) {
        $query = "DELETE FROM Documents WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }
}
?> 