<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Car Dealership</h1>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <h2 class="text-xl font-semibold mb-3">Test</h2>
        <div class="mt-4">
            <button id="loadData" class="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600">
                Test AJAX Connection
            </button>
        </div>
        <div id="result" class="mt-4"></div>
    </div>
</div>

<script>
document.getElementById('loadData').addEventListener('click', function() {
    fetch('api/test.php')
        .then(response => response.json())
        .then(data => {
            document.getElementById('result').innerHTML = 
                `<div class="bg-green-100 border border-green-200 text-green-800 p-3 rounded">
                    AJAX working! Received: ${data.message}<br>
                    Timestamp: ${data.timestamp}
                </div>`;
        })
        .catch(error => {
            document.getElementById('result').innerHTML = 
                `<div class="bg-red-100 border border-red-200 text-red-800 p-3 rounded">
                    Error: ${error.message}
                </div>`;
        });
});
</script>

<?php include_once 'includes/footer.php'; ?> 