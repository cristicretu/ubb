<?php
class Property {
    private $conn;
    private $table_name = "Property";

    public $id;
    public $address;
    public $description;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, address, description FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function searchAll($description) {
      $query = "SELECT id, address, description FROM " . $this->table_name . " WHERE description LIKE ?";
      $stmt = $this->conn->prepare($query);

      $cleanDescription = htmlspecialchars(strip_tags($description));
      // Add wildcards for partial matching
      $searchTerm = "%" . $cleanDescription . "%";
      $stmt->bindParam(1, $searchTerm);
      
      if ($stmt->execute()) {
          return $stmt;
      }
      
      return false;  
    }


    public function create($address, $description) {
        $query = "INSERT INTO " . $this->table_name . " (address, description) VALUES (?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanName = htmlspecialchars(strip_tags($address));
        $cleanSecretQuestion = htmlspecialchars(strip_tags($description));
        
        $stmt->bindParam(1, $cleanName);
        $stmt->bindParam(2, $cleanSecretQuestion);

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
}
?>