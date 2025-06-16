<?php
class UserToProperties {
    private $conn;
    private $table_name = "UserToProperties";

    public $id;
    public $idUser;
    public $isPrperty;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, idUser, idProperty FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function readAllByUserID($userId) {
        $query = "SELECT utp.id, utp.idUser, utp.idProperty, p.address, p.description 
                  FROM " . $this->table_name . " utp 
                  JOIN Property p ON utp.idProperty = p.id 
                  WHERE utp.idUser = ?";
        
        $stmt = $this->conn->prepare($query);

        $stmt->bindParam(1, $userId);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function getPropertiesWithMoreThanOneOwner() {
        $query = "SELECT p.id, p.address, p.description, COUNT(*) as owner_count FROM " . $this->table_name . " utp JOIN Property p ON utp.idProperty = p.id GROUP BY p.id HAVING COUNT(*) > 1";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();

        return $stmt;
    }

    public function getNumberOfOwners($idProperty) {
        $query = "SELECT COUNT(*) FROM " . $this->table_name . " WHERE idProperty = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $idProperty);

        $stmt->execute();

        return $stmt->fetch(PDO::FETCH_ASSOC);   
    }


    public function create($idUser, $idProperty) {
        $query = "INSERT INTO " . $this->table_name . " (idUser, idProperty) VALUES (?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanName = htmlspecialchars(strip_tags($idUser));
        $cleanSecretQuestion = htmlspecialchars(strip_tags($idProperty));
        
        $stmt->bindParam(1, $cleanName);
        $stmt->bindParam(2, $cleanSecretQuestion);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }

    public function deleteById($id) {
        $query = "DELETE FROM " . $this->table_name . " WHERE id = ?";

        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $id);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }
}
?>