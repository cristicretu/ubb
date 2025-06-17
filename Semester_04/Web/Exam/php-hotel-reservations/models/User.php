<?php
class User {
    private $conn;
    private $table_name = "User";

    public $id;
    public $username;
    public $password;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, username, password FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function findOne($username) {
      $query = "Select id, username, password FROM " . $this->table_name . " WHERE username = ?";

      $stmt = $this->conn->prepare($query);
      $stmt->bindParam(1, $username);
      $stmt->execute();

      return $stmt;
    }

}
?>