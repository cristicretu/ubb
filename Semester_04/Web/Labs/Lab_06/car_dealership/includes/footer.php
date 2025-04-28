    </div>
    <footer class="mt-10 py-6">
        <div class="container mx-auto px-4">
            <div class="flex flex-col md:flex-row justify-between">
            </div>
            <div class="mt-6 border-t border-neutral-300 pt-4 text-sm text-neutral-400">
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