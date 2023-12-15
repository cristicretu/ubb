import tkinter as tk
from tkinter import (
    BOTH,
    END,
    LEFT,
    RIGHT,
    Y,
    messagebox,
    simpledialog,
    Listbox,
    Scrollbar,
)
from lib.helpers import read_string_tkinter, read_valid_integer_tkinter
from repository.repository_exception import RepositoryException

from services.book_service import BookService
from services.client_service import ClientService
from services.rental_service import RentalService


class BookLibraryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Library Management System")

        self.__book_service = BookService()
        self.__client_service = ClientService()
        self.__rental_service = RentalService()

        self.__rental_service.generate_random_rentals(
            self.__book_service.get_all_books(),
            self.__client_service.get_all_clients(),
        )

        self.main_menu()

    def main_menu(self):
        self.clear_frame()

        tk.Button(self.root, text="1. Book operations", command=self.book_menu).pack()
        tk.Button(
            self.root, text="2. Client operations", command=self.client_menu
        ).pack()
        tk.Button(
            self.root, text="3. Rental operations", command=self.rental_menu
        ).pack()
        tk.Button(self.root, text="4. Statistics", command=self.statistics_menu).pack()
        tk.Button(self.root, text="0. Exit", command=self.root.quit).pack()

    def clear_frame(self):
        for widget in self.root.winfo_children():
            widget.destroy()

    def book_menu(self):
        self.clear_frame()

        tk.Button(self.root, text="1. Add book", command=self.add_book).pack()
        tk.Button(self.root, text="2. Remove book", command=self.remove_book).pack()
        tk.Button(self.root, text="3. Update book", command=self.update_book).pack()
        tk.Button(self.root, text="4. List books", command=self.list_books).pack()
        tk.Button(self.root, text="0. Back", command=self.main_menu).pack()

    def add_book(self):
        self.clear_frame()

        book_id = simpledialog.askinteger("Book ID", "Enter book ID")
        if book_id is None:
            self.book_menu()
            return

        title = simpledialog.askstring("Title", "Enter book title")
        if title is None:
            self.book_menu()
            return

        author = simpledialog.askstring("Author", "Enter book author")
        if author is None:
            self.book_menu()
            return

        try:
            self.__book_service.add_book(book_id, title, author, testing=True)
            messagebox.showinfo("Success", "Book added successfully!")

            self.book_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))

            self.book_menu()

    def remove_book(self):
        self.clear_frame()

        book_id = simpledialog.askinteger("Book ID", "Enter book ID")
        if book_id is None:
            self.book_menu()
            return

        try:
            self.__book_service.remove_book(book_id, testing=True)
            messagebox.showinfo("Success", "Book removed successfully!")

            self.book_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))

            self.book_menu()

    def update_book(self):
        self.clear_frame()

        book_id = simpledialog.askinteger("Book ID", "Enter book ID")
        if book_id is None:
            self.book_menu()
            return

        title = simpledialog.askstring("Title", "Enter book title")
        if title is None:
            self.book_menu()
            return

        author = simpledialog.askstring("Author", "Enter book author")
        if author is None:
            self.book_menu()
            return

        try:
            self.__book_service.update_book(book_id, title, author, testing=True)
            messagebox.showinfo("Success", "Book updated successfully!")

            self.book_menu()
        except Exception as exception:
            messagebox.showerror("Error", str(exception))
            self.book_menu()

    def list_books(self):
        self.clear_frame()

        books = self.__book_service.get_all_books()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for book in books:
            listbox.insert(END, book)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.book_menu).pack()

    def client_menu(self):
        self.clear_frame()

        tk.Button(self.root, text="1. Add client", command=self.add_client).pack()
        tk.Button(self.root, text="2. Remove client", command=self.remove_client).pack()
        tk.Button(self.root, text="3. Update client", command=self.update_client).pack()
        tk.Button(self.root, text="4. List clients", command=self.list_clients).pack()
        tk.Button(self.root, text="0. Back", command=self.main_menu).pack()

    def add_client(self):
        self.clear_frame()

        client_id = simpledialog.askinteger("Client ID", "Enter client ID")
        if client_id is None:
            self.client_menu()
            return

        name = simpledialog.askstring("Name", "Enter client name")
        if name is None:
            self.client_menu()
            return

        try:
            self.__client_service.add_client(client_id, name, testing=True)
            messagebox.showinfo("Success", "Client added successfully!")

            self.client_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))
            self.client_menu()

    def remove_client(self):
        self.clear_frame()

        client_id = simpledialog.askinteger("Client ID", "Enter client ID")
        if client_id is None:
            self.client_menu()
            return

        try:
            self.__client_service.remove_client(client_id, testing=True)
            messagebox.showinfo("Success", "Client removed successfully!")

            self.client_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))
            self.client_menu()

    def update_client(self):
        self.clear_frame()

        client_id = simpledialog.askinteger("Client ID", "Enter client ID")
        if client_id is None:
            self.client_menu()
            return

        name = simpledialog.askstring("Name", "Enter client name")
        if name is None:
            self.client_menu()
            return

        try:
            self.__client_service.update_client(client_id, name, testing=True)
            messagebox.showinfo("Success", "Client updated successfully!")

            self.client_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))
            self.client_menu()

    def list_clients(self):
        self.clear_frame()

        clients = self.__client_service.get_all_clients()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for client in clients:
            listbox.insert(END, client)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.client_menu).pack()

    def rental_menu(self):
        self.clear_frame()

        tk.Button(self.root, text="1. Rent book", command=self.rent_book).pack()
        tk.Button(self.root, text="2. Return book", command=self.return_book).pack()
        tk.Button(self.root, text="3. List rentals", command=self.list_rentals).pack()
        tk.Button(self.root, text="0. Back", command=self.main_menu).pack()

    def rent_book(self):
        self.clear_frame()

        rental_id = simpledialog.askstring("Rental ID", "Enter rental ID")
        if rental_id is None:
            self.rental_menu()
            return

        book_id = simpledialog.askinteger("Book ID", "Enter book ID")
        if book_id is None:
            self.rental_menu()
            return

        client_id = simpledialog.askinteger("Client ID", "Enter client ID")
        if client_id is None:
            self.rental_menu()
            return

        rented_date = simpledialog.askstring("Rented date", "Enter rented date")
        if rented_date is None:
            self.rental_menu()
            return

        books = self.__book_service.get_all_books()
        clients = self.__client_service.get_all_clients()

        try:
            self.__rental_service.add_rental(
                rental_id, book_id, client_id, rented_date, books, clients
            )
            messagebox.showinfo("Success", "Book rented successfully!")

            self.rental_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))
            self.rental_menu()

    def return_book(self):
        self.clear_frame()

        rental_id = read_string_tkinter("Rental ID", "Enter rental ID")
        if rental_id is None:
            self.rental_menu()
            return

        returned_date = read_valid_integer_tkinter(
            "Returned date", "Enter returned date"
        )
        if returned_date is None:
            self.rental_menu()
            return

        try:
            self.__rental_service.update_rental(rental_id, returned_date)
            messagebox.showinfo("Success", "Book returned successfully!")

            self.rental_menu()
        except RepositoryException as exception:
            messagebox.showerror("Error", str(exception))
            self.rental_menu()

    def list_rentals(self):
        self.clear_frame()

        rentals = self.__rental_service.get_all_rentals()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for rental in rentals:
            listbox.insert(END, rental)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.rental_menu).pack()

    def statistics_menu(self):
        self.clear_frame()

        tk.Button(
            self.root, text="1. Most rented books", command=self.most_rented_books
        ).pack()
        tk.Button(
            self.root, text="2. Most active clients", command=self.most_active_clients
        ).pack()
        tk.Button(
            self.root, text="3. Most rented author", command=self.most_rented_author
        ).pack()
        tk.Button(self.root, text="0. Back", command=self.main_menu).pack()

    def most_rented_books(self):
        self.clear_frame()

        books = self.__book_service.get_all_books()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for book in self.__rental_service.sort_rentals_by_most_rented_book(books):
            listbox.insert(END, book)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.statistics_menu).pack()

    def most_active_clients(self):
        self.clear_frame()

        clients = self.__client_service.get_all_clients()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for client in self.__rental_service.sort_rentals_by_most_active_client(clients):
            listbox.insert(END, client)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.statistics_menu).pack()

    def most_rented_author(self):
        self.clear_frame()

        books = self.__book_service.get_all_books()

        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        for author in self.__rental_service.sort_rentals_by_most_rented_author(books):
            listbox.insert(END, author)
        listbox.pack(side=LEFT, fill=BOTH)

        scrollbar.config(command=listbox.yview)

        tk.Button(self.root, text="Back", command=self.statistics_menu).pack()
