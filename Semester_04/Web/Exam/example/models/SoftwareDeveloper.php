<?php
class Car {
    private $conn;
    private $table_name = "SoftwareDeveloper";

    public $id;
    public $name;
    public $age;
    public $skills;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, name, age, skills FROM " . $this->table_name;
        
        $query .= " ORDER BY created_at DESC";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }


    public function create() {
        $query = "INSERT INTO " . $this->table_name . " SET name = ?, age = ?, skills = ?";

        $stmt = $this->conn->prepare($query);
        
        $this->name = htmlspecialchars(strip_tags($this->name));
        $this->age = htmlspecialchars(strip_tags($this->age));
        $this->skills = htmlspecialchars(strip_tags($this->skills));
        $stmt->bindParam(1, $this->name);
        $stmt->bindParam(2, $this->age);
        $stmt->bindParam(3, $this->skills);

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
        $query = "UPDATE " . $this->table_name . " SET name = ?, age = ?, skills = ? WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->name);
        $stmt->bindParam(2, $this->age);
        $stmt->bindParam(3, $this->skills);
        $stmt->bindParam(4, $this->id);
        if($stmt->execute()) {
            return true;
        }
    }

}
?>