<?php
class Reservation {
    private $conn;
    private $table_name = "Reservation";

    public $id;
    public $userId;
    public $roomId;
    public $checkInDate;
    public $checkOutDate;
    public $numberOfGuests;
    public $totalPrice;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function findOne($id) {
        $query = "SELECT id, userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice FROM " . $this->table_name . " WHERE id = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $id);

        $stmt->execute();

        return $stmt->fetch(PDO::FETCH_ASSOC);
    }

    public function readAllByRoomId($roomId) {
        $query = "SELECT id, userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice FROM " . $this->table_name . " WHERE roomId = ?";
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->bindParam(1, $roomId);

        $stmt->execute();

        return $stmt;
    }

    public function isReserved($roomId, $starDate, $endDate) {
        $query = "select * from " . $this->table_name . " where ? < checkOutDate and checkInDate < ?";

        $stmt = $this->conn->prepare($query);

        $stmt->bindParam(1, $starDate);
        $stmt->bindParam(2, $endDate);

        $stmt->execute();

        return $stmt->fetch();
    } 


    public function create($userId, $roomId, $checkInDate, $checkOutDate, $totalPrice) {
        $query = "INSERT INTO " . $this->table_name . " (userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice) VALUES (?, ?, ?, ?, ?, ?)";
        
        $stmt = $this->conn->prepare($query);

        $numberOfGuests = 1;
        
        $stmt->bindParam(1, $userId);
        $stmt->bindParam(2, $roomId);
        $stmt->bindParam(3, $checkInDate);
        $stmt->bindParam(4, $checkOutDate);
        $stmt->bindParam(5, $numberOfGuests);
        $stmt->bindParam(6, $totalPrice);

        $stmt->execute();
    }

    // public function create($address, $description) {
    //     $query = "INSERT INTO " . $this->table_name . " (address, description) VALUES (?, ?)";

    //     $stmt = $this->conn->prepare($query);
        
    //     $cleanAddress = htmlspecialchars(strip_tags($address));
    //     $cleanDescription = htmlspecialchars(strip_tags($description));
        
    //     $stmt->bindParam(1, $cleanAddress);
    //     $stmt->bindParam(2, $cleanDescription);

    //     if($stmt->execute()) {
    //         return $this->conn->lastInsertId();
    //     }

    //     return false;
    // }

    // public function deleteById($id) {
    //     $query = "DELETE FROM " . $this->table_name . " WHERE id = ?";

    //     $stmt = $this->conn->prepare($query);
        
    //     $stmt->bindParam(1, $id);

    //     if($stmt->execute()) {
    //         return true;
    //     }

    //     return false;
    // }
}
?>