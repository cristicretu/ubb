<?php
require_once 'config/database.php';

class Post {
    public $id;
    public $user;
    public $topicId;
    public $text;
    public $date;

    private $db;

    public function __construct() {
        $this->db = Database::getInstance()->getConnection();
    }

    public function getAllPosts() {
        $query = "SELECT * FROM Posts";
        $stmt = $this->db->prepare($query);
        $stmt->execute();
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function getPostsByUser($user) {
        $query = "SELECT * FROM Posts WHERE user = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$user]);
        return $stmt->fetchAll(PDO::FETCH_ASSOC);
    }

    public function addPost($user, $topicId, $text, $date) {
        $query = "INSERT INTO Posts (user, topicid, text, date) VALUES (?, ?, ?, ?)";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$user, $topicId, $text, $date]);
    }

    public function updatePost($id, $topicId, $text) {
        $query = "UPDATE Posts SET text = ?, topicid = ? WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$text, $topicId, $id]);
    }

    public function getPostById($id) {
        $query = "SELECT * FROM Posts WHERE id = ?";
        $stmt = $this->db->prepare($query);
        $stmt->execute([$id]);
        return $stmt->fetch(PDO::FETCH_ASSOC);
    }

    public function deletePost($id) {
        $query = "DELETE FROM Posts WHERE id = ?";
        $stmt = $this->db->prepare($query);
        return $stmt->execute([$id]);
    }
}
?> 