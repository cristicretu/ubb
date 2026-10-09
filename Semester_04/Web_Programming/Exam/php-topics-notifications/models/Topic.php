<?php
require_once 'config/database.php';

class Topic {
    public $id;
    public $tipicname;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
    }

    public function getAllTopics() {
        $query = "SELECT * FROM Topics";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function getTopicByName($tipicname) {
        $query = "SELECT * FROM Topics WHERE tipicname = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$tipicname]);
        return $stmt->fetch(PDO::FETCH_ASSOC);
    }

    public function addTopic($tipicname) {
        $query = "INSERT INTO Topics (tipicname) VALUES (?)";
        $stmt = $this->db->prepare($query);
        if ($stmt->execute([$tipicname])) {
            return $this->db->lastInsertId();
        }
        return false;
    }

    public function getTopicById($id) {
        $query = "SELECT * FROM Topics WHERE id = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$id]);
        return $stmt->fetch(PDO::FETCH_ASSOC);
    }
}
?> 