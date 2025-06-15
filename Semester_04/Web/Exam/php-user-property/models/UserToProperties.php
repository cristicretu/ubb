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