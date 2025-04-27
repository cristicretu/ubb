    </div>
    <footer class="bg-neutral-800 text-white mt-10 py-6">
        <div class="container mx-auto px-4">
            <div class="flex flex-col md:flex-row justify-between">
                <div class="mb-4 md:mb-0">
                    <h2 class="text-lg font-bold mb-2">Car Dealership</h2>
                </div>
                
            </div>
            <div class="mt-6 border-t border-neutral-700 pt-4 text-sm text-neutral-400">
                <p>&copy; <?php echo date("Y"); ?> Car Dealership. All rights reserved.</p>
            </div>
        </div>
    </footer>

    <script>
        document.getElementById('menu-toggle').addEventListener('click', function() {
            const mobileMenu = document.getElementById('mobile-menu');
            mobileMenu.classList.toggle('hidden');
        });
    </script>
</body>
</html> 