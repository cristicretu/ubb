<?php
class Database {
    private static $instance = null;
    private $pdo;
    private $db_path = 'tezt';

    private function __construct() {
        try {
            $this->pdo = new PDO("sqlite:" . $this->db_path);
            $this->pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
            $this->initializeDatabase();
        } catch (PDOException $e) {
            throw new RuntimeException("Database connection failed: " . $e->getMessage());
        }
    }

    public static function getInstance() {
        if (self::$instance === null) {
            self::$instance = new self();
        }
        return self::$instance;
    }

    public function getConnection() {
        return $this->pdo;
    }

    private function initializeDatabase() {
    }

    public function closeConnection() {
        $this->pdo = null;
    }
}
?> 