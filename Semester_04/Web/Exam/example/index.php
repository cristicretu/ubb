<?php
include_once 'config/database.php';
include_once 'includes/header.php';
?>

<div class="mb-6">
    <h1 class="text-3xl font-bold text-neutral-800 mb-4"> Project Management</h1>
    
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
</script>

<?php include_once 'includes/footer.php'; ?> 