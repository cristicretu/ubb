<?php
class Project {
    private $conn;
    private $table_name = "Project";

    public $id;
    public $ProjectManagerID;
    public $name;
    public $description;
    public $members;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, ProjectManagerID, name, description, members FROM " . $this->table_name;
        
        $query .= " ORDER BY created_at DESC";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function readAllByProjectManagerID($projectManagerID) {
        $query = "SELECT id, ProjectManagerID, name, description, members FROM " . $this->table_name . " WHERE ProjectManagerID = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $projectManagerID);

        $stmt->execute();

        return $stmt;
    }


    public function create() {
        $query = "INSERT INTO " . $this->table_name . " SET ProjectManagerID = ?, name = ?, description = ?, members = ?";

        $stmt = $this->conn->prepare($query);
        
        $this->ProjectManagerID = htmlspecialchars(strip_tags($this->ProjectManagerID));
        $this->name = htmlspecialchars(strip_tags($this->name));
        $this->description = htmlspecialchars(strip_tags($this->description));
        $this->members = htmlspecialchars(strip_tags($this->members));
        $stmt->bindParam(1, $this->ProjectManagerID);
        $stmt->bindParam(2, $this->name);
        $stmt->bindParam(3, $this->description);
        $stmt->bindParam(4, $this->members);

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
        $query = "UPDATE " . $this->table_name . " SET ProjectManagerID = ?, name = ?, description = ?, members = ? WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $this->ProjectManagerID);
        $stmt->bindParam(2, $this->name);
        $stmt->bindParam(3, $this->description);
        $stmt->bindParam(4, $this->members);
        $stmt->bindParam(5, $this->id);
        if($stmt->execute()) {
            return true;
        }

        return false;
    }

}
?>