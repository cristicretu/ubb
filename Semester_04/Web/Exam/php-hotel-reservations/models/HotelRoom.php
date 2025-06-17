<?php
include_once 'Reservations.php';

class HotelRoom {
    
    private $conn;
    private $table_name = "HotelRoom";


    public $id;
    public $roomNumber;
    public $capacity;
    public $basePrice;

    public function __construct($db) {
        $this->conn = $db;
    }

    public function readAll() {
        $query = "SELECT id, roomNumber, capacity, basePrice FROM " . $this->table_name;
        
        $stmt = $this->conn->prepare($query);
        
        $stmt->execute();
        
        return $stmt;
    }

    public function create($roomNumber, $capacity, $basePrice) {
        $query = "INSERT INTO " . $this->table_name . " (roomNumber, capacity, basePrice) VALUES (?, ?, ?)";

        $stmt = $this->conn->prepare($query);
        
        $cleanRoomNumber = htmlspecialchars(strip_tags($roomNumber));
        $cleanCapacity = htmlspecialchars(strip_tags($capacity));
        $cleanBasePrice = htmlspecialchars(strip_tags($basePrice));
        
        $stmt->bindParam(1, $cleanRoomNumber);
        $stmt->bindParam(2, $cleanCapacity);
        $stmt->bindParam(3, $cleanBasePrice);

        if($stmt->execute()) {
            return true;
        }

        return false;
    }


    public function readAvailableRooms($start_date, $end_date) {
        // first get all rooms
        // then for each of these rooms, find all possible reservations.
        // then check for all possible overlap cases
        $allRooms = $this->readAll();
        $availableRooms = [];
        $reservation = new Reservation($this->conn);

        while ($row = $allRooms->fetch()) {
            $roomId = $row['id'];
            $roomNumber = $row['roomNumber'];
            $capacity = $row['capacity'];
            $basePrice = $row['basePrice'];

            $reservations = $reservation->readAllByRoomId($roomId);

            $available = true;
            while ($reservationRow = $reservations->fetch()) {
                $checkInDate = $reservationRow['checkInDate'];
                $checkOutDate = $reservationRow['checkOutDate'];

                if ($start_date < $checkOutDate && $checkInDate < $end_date) {
                    $available = false;
                    break;
                }
            }

            if ($available) {
                $availableRooms[] = $row;
            }
        }

        return $availableRooms;
    }


}
?>