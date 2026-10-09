<?php
session_start();
include_once 'config/database.php';
include_once 'models/User.php';
include_once 'models/HotelRoom.php';
include_once 'includes/header.php';

if (!isset($_SESSION['currentUser'])) {
    header('Location: login.php');
    exit();
}

$currentUser = $_SESSION['currentUser'];
$userId = $_SESSION['userId'];
$database = new Database();
$db = $database->getConnection();
$hotelRoom = new HotelRoom($db);
$reservation = new Reservation($db);
$success_message = '';
$error_message = '';

$availableRooms = [];
$reservations = [];
$totalGuests = 0;

$reservationsStmt = $reservation->readAllByUserId($userId);

while ($reservationRow = $reservationsStmt->fetch()) {
    $reservations[] = $reservationRow;
}


if ($_POST) {
    if (isset($_POST['view_available'])) {
        $start_date = $_POST['start_date'];
        $end_date = $_POST['end_date'];

        $availableRooms = $hotelRoom->readAvailableRooms($start_date, $end_date);

        if ($availableRooms) {
            $success_message = 'Available rooms: ' . count($availableRooms);
        } else {
            $error_message = 'No available rooms';
        }
    } else if (isset($_POST['book_room'])) {
        $room_id = $_POST['room_id'];
        $start_date = $_POST['start_date'];
        $end_date = $_POST['end_date'];

        $booked = $hotelRoom->bookRoom($userId, $room_id, $start_date, $end_date);
        if ($booked) {
            $success_message = 'Room booked successfully';
        } else {
            $error_message = 'Room already booked or not available';
        }
    } else if (isset($_POST['total_guests'])) {
        $date = $_POST['date'];
        $totalGuests = $reservation->getTotalGuests($date);
        $success_message = 'Total number of guests in a day: ' . $totalGuests;
    }
}


?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Propety Management</h1>
    <p class="text-gray-600 mb-4">Welcome, <?php echo htmlspecialchars($currentUser); ?>! 
        <a href="logout.php" class="text-blue-500 hover:underline">Logout</a>
    </p>
    
    <?php if (!empty($success_message)): ?>
        <div class="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($success_message); ?>
        </div>
    <?php endif; ?>
    
    <?php if (!empty($error_message)): ?>
        <div class="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
            <?php echo htmlspecialchars($error_message); ?>
        </div>
    <?php endif; ?>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">


        <h2 class="text-2xl font-bold">See available rooms in between 2 dates</h2>
        <form method="post" action="index.php" class="flex flex-col gap-2">
            <input
            type="date"
            id="start"
            name="start_date"
            class="border border-gray-300 rounded-md p-2"
            />

            <input
            type="date"
            id="end"
            name="end_date"
            class="border border-gray-300 rounded-md p-2"
            />

            <button type="submit" name="view_available" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">
                Search
            </button>
        </form>

        <?php if ($availableRooms): ?>
            <table class="w">
                <thead>
                    <tr>
                        <th>Room Number</th>
                        <th>Capacity</th>
                        <th>Base Price</th>
                        <th>Book</th>
                    </tr>
                </thead>
                <tbody>
                    <?php foreach ($availableRooms as $room): ?>
                        <tr>
                            <form method="post" action="index.php">
                                <input type="hidden" name="room_id" value="<?php echo $room['id']; ?>">
                                <input type="hidden" name="start_date" value="<?php echo $start_date; ?>">
                                <input type="hidden" name="end_date" value="<?php echo $end_date; ?>">
                                <td><input disabled name="room_id" value="<?php echo $room['roomNumber']; ?>"></td>
                                <td><input disabled name="start_date" value="<?php echo $room['capacity']; ?>"></td>
                                <td><input disabled name="end_date" value="<?php echo $room['basePrice']; ?>"></td>
                            <td><button type="submit" name="book_room" class="bg-green-500 hover:bg-green-600 text-white px-4 py-2 rounded-md">
                                Book
                            </button></td>
                            </form>
                        </tr>
                    <?php endforeach; ?>
                </tbody>
            </table>
        <?php endif; ?>

        <h2 class="text-2xl font-bold">Total number of guests in a day</h2>
        <form method="post" action="index.php" class="flex flex-col gap-2">
            <input type="date" name="date" class="border border-gray-300 rounded-md p-2">
            <button type="submit" name="total_guests" class="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md">
                Calculate
            </button>
        </form>

        <?php if ($totalGuests != 0): ?>
            <p>Total number of guests in a day: <?php echo $totalGuests; ?></p>
        <?php endif; ?>
        
        
         <h2 class="text-2xl font-bold">My reservations</h2>
         <table class="w">
            <thead>
                <tr>
                    <th>Check In Date</th>
                    <th>Check Out Date</th>
                    <th>Total Price</th>
                </tr>
            </thead>
            <tbody>
                <?php foreach ($reservations as $reservation): ?>
                    <tr>
                        <td><?php echo $reservation['checkInDate']; ?></td>
                        <td><?php echo $reservation['checkOutDate']; ?></td>
                        <td><?php echo $reservation['totalPrice']; ?></td>
                    </tr>
                <?php endforeach; ?>
            </tbody>
         </table>




        </div>
    </div>
</div>