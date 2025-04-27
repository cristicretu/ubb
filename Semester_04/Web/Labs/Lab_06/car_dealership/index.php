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
    </div>
</div>

<script>
document.addEventListener('DOMContentLoaded', function() {
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
                    
                    if (index === 0) {
                        tabElement.classList.add('border-b-2', 'border-blue-500', 'text-blue-600', 'font-medium');
                        document.getElementById('category-content').innerHTML = `
                            <h3 class="text-xl font-semibold mb-2">${category.name}</h3>
                            <p class="text-gray-700">${category.description || 'No description available'}</p>
                        `;
                    } else {
                        tabElement.classList.add('text-gray-500', 'hover:text-gray-700');
                    }
                    
                    tabElement.addEventListener('click', function() {
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