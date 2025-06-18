<?php
class Websites {
    private $conn;
    private $table_name = "Websites";

    public $id;
    public $URL;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, URL FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }


    public function readAllWithDocumentsCount() {
        $query = "SELECT Websites.id, Websites.URL, COUNT(Documents.id) AS document_count FROM Websites LEFT JOIN Documents ON Websites.id = Documents.idWebsite GROUP BY Websites.id";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

}
?>