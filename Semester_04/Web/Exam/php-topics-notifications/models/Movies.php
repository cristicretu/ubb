<?php
require_once 'config/database.php';

class Movies {
    public $id;
    public $title;
    public $genre;
    public $year;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
        $this->initializeTable();
    }

    private function initializeTable() {
        $query = "CREATE TABLE IF NOT EXISTS Movies (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            title VARCHAR(255),
            genre VARCHAR(255),
            year INTEGER
        )";
        $this->db->exec($query);
    }

    public function getAllMovies() {
        $query = "SELECT id, title, genre, year FROM Movies";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function createMovie($title, $genre, $year) {
        $query = "INSERT INTO Movies (title, genre, year) VALUES (?, ?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$title, $genre, $year]);
    }

    public function updateMovie($id, $title, $genre, $year) {
        $query = "UPDATE Movies SET title = ?, genre = ?, year = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$title, $genre, $year, $id]);
    }

    public function deleteMovie($id) {
        $query = "DELETE FROM Movies WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }
}
?> 