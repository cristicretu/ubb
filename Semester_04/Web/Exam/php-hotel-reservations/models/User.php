<?php
class User {
    private $conn;
    private $table_name = "User";

    public $id;
    public $name;
    public $secretQuestion;
    public $secretAnswer;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, name, secretQuestion, secretAnswer FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function findOne($username) {
      $query = "Select id, name, secretQuestion, secretAnswer FROM " . $this->table_name . " WHERE name = ?";

      $stmt = $this->conn->prepare($query);
      $stmt->bindParam(1, $username);
      $stmt->execute();

      return $stmt;
    }


    public function create($name, $secretQuestion, $secretAnswer) {
        $query = "INSERT INTO " . $this->table_name . " (name, secretQuestion, secretAnswer) VALUES (?, ?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanName = htmlspecialchars(strip_tags($name));
        $cleanSecretQuestion = htmlspecialchars(strip_tags($secretQuestion));
        $cleanSecretAnswer = htmlspecialchars(strip_tags($secretAnswer));
        
        $stmt->bindParam(1, $cleanName);
        $stmt->bindParam(2, $cleanSecretQuestion);
        $stmt->bindParam(3, $cleanSecretAnswer);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }

    public function delete() {
        $query = "DELETE FROM " . $this->table_name . " WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->id);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }

    public function update() {
        $query = "UPDATE " . $this->table_name . " SET name = ?, secretQuestion = ?, secretAnswer = ? WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->name);
        $stmt->bindParam(2, $this->secretQuestion);
        $stmt->bindParam(3, $this->secretAnswer);
        $stmt->bindParam(4, $this->id);
        if($stmt->execute()) {
            return true;
        }

        return false;
    }

}
?>