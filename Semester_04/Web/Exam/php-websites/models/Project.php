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
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function readAllByProjectManagerID($projectManagerID) {
        if ($projectManagerID == null) {
            return $this->readAll();
        }
        
        $query = "SELECT id, ProjectManagerID, name, description, members FROM " . $this->table_name . " WHERE ProjectManagerID = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $projectManagerID);

        $stmt->execute();

        return $stmt;
    }

    public function assignProject($projectName, $projectManagerID) {
        // Check if project already exists
        $query = "SELECT id FROM " . $this->table_name . " WHERE name = ?";
        $stmt = $this->conn->prepare($query);
        $stmt->bindParam(1, $projectName);
        $stmt->execute();
        $row = $stmt->fetch(PDO::FETCH_ASSOC);
        
        if (!$row) {
            // Create new project
            if ($this->create($projectName, $projectManagerID)) {
                $projectId = $this->conn->lastInsertId();
            } else {
                error_log("Failed to create project: $projectName with manager ID: $projectManagerID");
                return null;
            }
        } else {
            // Update existing project
            $query = "UPDATE " . $this->table_name . " SET ProjectManagerID = ? WHERE id = ?";
            $stmt = $this->conn->prepare($query);
            $stmt->bindParam(1, $projectManagerID);
            $stmt->bindParam(2, $row['id']);
            if (!$stmt->execute()) {
                error_log("Failed to update project: $projectName with manager ID: $projectManagerID");
                return null;
            }
            $projectId = $row['id'];
        }
        
        // Return the project details
        $query = "SELECT id, ProjectManagerID, name, description, members FROM " . $this->table_name . " WHERE id = ?";
        $stmt = $this->conn->prepare($query);
        $stmt->bindParam(1, $projectId);
        if (!$stmt->execute()) {
            error_log("Failed to fetch project details for ID: $projectId");
            return null;
        }
        
        return $stmt;
    }


    public function create($projectName, $projectManagerID) {
        $query = "INSERT INTO " . $this->table_name . " (ProjectManagerID, name, description, members) VALUES (?, ?, ?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanProjectManagerID = htmlspecialchars(strip_tags($projectManagerID));
        $cleanProjectName = htmlspecialchars(strip_tags($projectName));
        $cleanDescription = htmlspecialchars(strip_tags(''));
        $cleanMembers = htmlspecialchars(strip_tags(''));
        
        $stmt->bindParam(1, $cleanProjectManagerID);
        $stmt->bindParam(2, $cleanProjectName);
        $stmt->bindParam(3, $cleanDescription);
        $stmt->bindParam(4, $cleanMembers);

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