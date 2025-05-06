<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4">Car Dealership</h1>
    
    <div class="bg-white p-4 rounded shadow-md mb-6">
        <div class="overflow-x-auto">
            <div id="categories-tabs" class="flex space-x-2  mb-4 pb-1 whitespace-nowrap">
                <div class="px-4 py-2 text-center text-gray-500">Loading categories...</div>
            </div>
        </div>
        
        <div id="category-content" class="mt-4">
        </div>

        <div id="cars-container" class="mt-4">
        </div>
    </div>
</div>

<script>

let cars = [];

let USDollar = new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency: 'USD',
});

function loadCarsFromCategory(categoryId) {
    fetch(`api/cars/read.php?category_id=${categoryId}`)
        .then(response => response.json())
        .then(data => {
            cars = data.records;
            renderCars();
        })
        .catch(error => {
            console.error('Error loading cars:', error);
        });
}

function renderCars() {
    const carsContainer = document.getElementById('cars-container');
    carsContainer.innerHTML = '';

    cars.forEach(car => {
        const carElement = document.createElement('div');
        const features = car.features ? car.features.split(',') : [];
        carElement.className = 'bg-white p-4 rounded-lg border border-gray-200 mb-4';
        carElement.innerHTML = `
            <h3 class="text-lg font-semibold mb-2">${car.model}</h3>
            <div class="grid grid-cols-2 gap-2 mt-2">
                <div class="flex items-center">
                    <span class="text-gray-500 mr-2">Engine:</span>
                    <span class="font-medium">${car.engine_power}</span>
                </div>
                <div class="flex items-center">
                    <span class="text-gray-500 mr-2">Fuel:</span>
                    <span class="font-medium">${car.fuel_type}</span>
                </div>
                <div class="flex items-center">
                    <span class="text-gray-500 mr-2">Color:</span>
                    <span class="font-medium">${car.color}</span>
                </div>
                <div class="flex items-center">
                    <span class="text-gray-500 mr-2">Year:</span>
                    <span class="font-medium">${car.year}</span>
                </div>

                ${features.map(feature => `<span class="text-gray-500 mr-2">${feature}</span>`).join('')}
            </div>
            <div class="mt-3 flex justify-between items-center">
                <span class="text-lg font-bold text-green-600">${USDollar.format(car.price)}</span>
                <div class="flex items-center gap-2">
                    <button class="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 transition" onclick="window.location.href='views/edit_car.php?id=${car.id}'">Edit</button>
                    <button class="px-3 py-1 bg-red-500 text-white rounded hover:bg-red-600 transition" onclick="confirmDeleteCar(${car.id})">Delete</button>
                </div>
            </div>
        `;
        carsContainer.appendChild(carElement);
    });
}

function confirmDeleteCar(carId) {
    if (confirm('Are you sure you want to delete this car?')) {
        fetch(`api/cars/delete.php?id=${carId}`, {
            method: 'DELETE'
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                const activeTab = document.querySelector('#categories-tabs .border-blue-500');
                if (activeTab) {
                    const categoryId = activeTab.dataset.categoryId;
                    loadCarsFromCategory(categoryId);
                }
            } else {
                alert('Error deleting car: ' + data.message);
            }
        })
        .catch(error => {
            console.error('Error deleting car:', error);
            alert('Error deleting car. Please check console for details.');
        });
    }
}

function removeCars() {
    const carsContainer = document.getElementById('cars-container');
    carsContainer.innerHTML = '';
}

document.addEventListener('DOMContentLoaded', function() {
    const categoryId = new URLSearchParams(window.location.search).get('category_id') || 0;
    console.log(categoryId);
    
    fetch('api/categories.php')
        .then(response => response.json())
        .then(data => {
            if (data.records && data.records.length > 0) {
                const tabsContainer = document.getElementById('categories-tabs');
                tabsContainer.innerHTML = '';
                
                data.records.forEach((category, index) => {
                    const tabElement = document.createElement('div');
                    tabElement.className = 'cursor-pointer px-4 py-2 text-center transition-colors duration-200';
                    tabElement.dataset.categoryId = category.id;
                    tabElement.textContent = category.name;
                    
                    if (category.id == categoryId) {
                        tabElement.classList.add('border-b-2', 'border-blue-500', 'text-blue-600', 'font-medium');
                        document.getElementById('category-content').innerHTML = `
                            <h3 class="text-xl font-semibold mb-2">${category.name}</h3>
                            <p class="text-gray-700">${category.description || 'No description available'}</p>
                        `;
                        loadCarsFromCategory(category.id);
                    } else {
                        tabElement.classList.add('text-gray-500', 'hover:text-gray-700');
                    }
                    
                    tabElement.addEventListener('click', function() {
                        const currentActiveTab = document.querySelector('#categories-tabs .border-blue-500');
                        let previousCategory = '';
                        let previousId = '';
                        if (currentActiveTab) {
                            const prevCatId = currentActiveTab.dataset.categoryId;
                            previousCategory = currentActiveTab.textContent;
                            previousId = prevCatId;
                        }
                        
                        document.querySelectorAll('#categories-tabs > div').forEach(tab => {
                            tab.classList.remove('border-b-2', 'border-blue-500', 'text-blue-600', 'font-medium');
                            tab.classList.add('text-gray-500', 'hover:text-gray-700');
                        });
                        
                        this.classList.remove('text-gray-500', 'hover:text-gray-700');
                        this.classList.add('border-b-2', 'border-blue-500', 'text-blue-600', 'font-medium');
                        
                        document.getElementById('category-content').innerHTML = `
                            <h3 class="text-xl font-semibold mb-2">${category.name}</h3>
                            <p class="text-gray-700">${category.description || 'No description available'}</p>
                        `;
                        
                        if (previousCategory) {
                            const historyStack = JSON.parse(localStorage.getItem('categoryHistory') || '[]');
                            console.log(historyStack);
                            historyStack.push({
                                name: previousCategory,
                                id: previousId,
                                baseUrl: '<?php echo $base_url; ?>'
                            });
                            
                            localStorage.setItem('categoryHistory', JSON.stringify(historyStack));
                            
                            const prevPageLink = document.getElementById('previous-page');
                            if (prevPageLink) {
                                prevPageLink.textContent = `← ${previousCategory}`;
                                prevPageLink.href = '<?php echo $base_url; ?>index.php?category_id=' + previousId;
                            }
                        }
                        
                        const url = new URL(window.location);
                        url.searchParams.set('category_id', category.id);
                        window.history.replaceState({}, '', url);
                        
                        removeCars();
                        loadCarsFromCategory(category.id);
                    });
                    
                    tabsContainer.appendChild(tabElement);
                });
            } else {
                document.getElementById('categories-tabs').innerHTML = 
                    '<div class="px-4 py-2 text-center text-red-500">No categories found</div>';
            }
        })
        .catch(error => {
            document.getElementById('result').innerHTML = 
                `<div class="bg-red-100 border border-red-200 text-red-800 p-3 rounded">
                    Error loading categories: ${error.message}
                </div>`;
        });
});

</script>

<?php include_once 'includes/footer.php'; ?> 