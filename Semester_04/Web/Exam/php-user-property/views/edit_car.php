<?php
include_once '../config/database.php';
include_once '../models/Car.php';
include_once '../models/Category.php';
include_once '../includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Edit Car</h1>
    
    <div class="bg-white p-6 rounded-lg shadow-md">
        <form id="add-car-form" class="space-y-6">
            <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div>
                    <label for="model" class="block text-sm font-medium text-gray-700 mb-1">Car Model</label>
                    <input type="text" id="model" name="model" required 
                        class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                </div>
                
                <div>
                    <label for="year" class="block text-sm font-medium text-gray-700 mb-1">Year</label>
                    <input type="number" id="year" name="year" min="1900" max="2025" required
                        class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                </div>
                
                <div>
                    <label for="engine_power" class="block text-sm font-medium text-gray-700 mb-1">Engine Power</label>
                    <input type="text" id="engine_power" name="engine_power" required 
                        class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                </div>
                
                <div>
                    <label for="fuel_type" class="block text-sm font-medium text-gray-700 mb-1">Fuel Type</label>
                    <select id="fuel_type" name="fuel_type" required
                        class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                        <option value="">Select Fuel Type</option>
                        <option value="Gasoline">Gasoline</option>
                        <option value="Diesel">Diesel</option>
                        <option value="Electric">Electric</option>
                        <option value="Hybrid">Hybrid</option>
                        <option value="LPG">LPG</option>
                    </select>
                </div>
                
                <div>
                    <label for="price" class="block text-sm font-medium text-gray-700 mb-1">Price (USD)</label>
                    <div class="relative">
                        <span class="absolute inset-y-0 left-0 pl-3 flex items-center text-gray-500">$</span>
                        <input type="number" id="price" name="price" min="0" step="0.01" required
                            class="w-full pl-8 px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                    </div>
                </div>
                
                <div>
                    <label for="color" class="block text-sm font-medium text-gray-700 mb-1">Color</label>
                    <input type="text" id="color" name="color" required
                        class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                </div>
            </div>
            
            <div>
                <label for="category_id" class="block text-sm font-medium text-gray-700 mb-1">Category</label>
                <select id="category_id" name="category_id" required
                    class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent">
                    <option value="">Select Category</option>
                </select>
            </div>
            
            <div>
                <label for="history" class="block text-sm font-medium text-gray-700 mb-1">Vehicle History</label>
                <textarea id="history" name="history" rows="4"
                    class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"></textarea>
            </div>

            <div>
                <label for="features" class="block text-sm font-medium text-gray-700 mb-1">Features</label>
                <input type="text" id="features" name="features" rows="4"
                    class="w-full px-4 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"></input>
                <p class="text-xs text-gray-500">Separate features with commas</p>
            </div>
            
            <div class="flex justify-end space-x-3">
                <a href="../index.php" class="px-6 py-2 bg-gray-200 text-gray-700 rounded-md hover:bg-gray-300 transition">
                    Cancel
                </a>
                <button type="submit" class="px-6 py-2 bg-blue-500 text-white rounded-md hover:bg-blue-600 transition">
                    Save Car
                </button>
            </div>
        </form>
    </div>
</div>

<script>
const urlParams = new URLSearchParams(window.location.search);
const carId = urlParams.get('id');

if (!carId) {
    alert('Car ID is missing!');
    window.location.href = '../index.php';
}

function loadCarData() {
    fetch(`../api/cars/read_one.php?id=${carId}`)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                const car = data.record;
                document.getElementById('model').value = car.model;
                document.getElementById('year').value = car.year;
                document.getElementById('engine_power').value = car.engine_power;
                document.getElementById('fuel_type').value = car.fuel_type;
                document.getElementById('price').value = car.price;
                document.getElementById('color').value = car.color;
                document.getElementById('history').value = car.history || '';
                document.getElementById('features').value = car.features || '';
                
                window.carCategoryId = car.category_id;
            } else {
                alert('Error loading car data: ' + data.message);
                window.location.href = '../index.php';
            }
        })
        .catch(error => {
            console.error('Error loading car:', error);
            alert('Error loading car data. Please check console for details.');
        });
}

document.addEventListener('DOMContentLoaded', function() {
    loadCarData();
    
    fetch('../api/categories.php')
        .then(response => response.json())
        .then(data => {
            if (data.records && data.records.length > 0) {
                const categorySelect = document.getElementById('category_id');
                categorySelect.innerHTML = '<option value="">Select Category</option>';
                
                data.records.forEach(category => {
                    const option = document.createElement('option');
                    option.value = category.id;
                    option.textContent = category.name;
                    categorySelect.appendChild(option);
                });
                
                if (window.carCategoryId) {
                    categorySelect.value = window.carCategoryId;
                }
            }
        })
        .catch(error => {
            console.error('Error loading categories:', error);
        });
});

document.getElementById('add-car-form').addEventListener('submit', function(event) {
    event.preventDefault();
    
    const formData = new FormData();
    formData.append('id', carId);
    formData.append('model', document.getElementById('model').value);
    formData.append('engine_power', document.getElementById('engine_power').value);
    formData.append('fuel_type', document.getElementById('fuel_type').value);
    formData.append('price', document.getElementById('price').value);
    formData.append('color', document.getElementById('color').value);
    formData.append('year', document.getElementById('year').value);
    formData.append('history', document.getElementById('history').value);
    formData.append('category_id', document.getElementById('category_id').value);
    formData.append('features', document.getElementById('features').value);

    fetch('../api/cars/edit.php', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            alert('Car updated successfully');
            window.location.href = '../index.php';
        } else {
            alert('Error updating car: ' + data.message);
        }
    })
    .catch(error => {
        console.error('Error updating car:', error);
        alert('Error updating car. Please check console for details.');
    });
});
</script>

<?php include_once '../includes/footer.php'; ?> 