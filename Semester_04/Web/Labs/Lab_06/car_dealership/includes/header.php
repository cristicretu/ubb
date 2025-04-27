<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Car Dealership</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <link rel="stylesheet" href="assets/css/style.css">
</head>
<body class="bg-neutral-100">
    <nav class="bg-neutral-800 text-white shadow-md mb-6">
        <div class="container mx-auto px-4 py-3">
            <div class="flex justify-between items-center">
                <a class="text-xl font-bold" href="index.php">Car Dealership</a>
                <div class="hidden md:block">
                    <div class="flex space-x-4">
                        <a class="px-3 py-2 hover:bg-neutral-700 rounded" href="index.php">Browse Cars</a>
                        <a class="px-3 py-2 hover:bg-neutral-700 rounded" href="views/add_car.php">Add Car</a>
                        <a class="px-3 py-2 hover:bg-neutral-700 rounded" href="views/categories.php">Categories</a>
                    </div>
                </div>
                <div class="md:hidden">
                    <button id="menu-toggle" class="text-white hover:text-neutral-300">
                        <svg class="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                            <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 6h16M4 12h16M4 18h16"></path>
                        </svg>
                    </button>
                </div>
            </div>
            <div id="mobile-menu" class="md:hidden hidden mt-3">
                <a class="block px-3 py-2 hover:bg-neutral-700 rounded mt-1" href="index.php">Browse Cars</a>
                <a class="block px-3 py-2 hover:bg-neutral-700 rounded mt-1" href="views/add_car.php">Add Car</a>
                <a class="block px-3 py-2 hover:bg-neutral-700 rounded mt-1" href="views/categories.php">Categories</a>
            </div>
        </div>
    </nav>
    <div class="container mx-auto px-4"> 