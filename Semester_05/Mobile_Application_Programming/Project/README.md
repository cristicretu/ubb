# gicu mecanicu 

A mobile app to help you keep track of your car's maintenance and important deadlines. Never forget when your car itp is due, when to renew your rovinieta, or how many miles you've driven. The app helps you stay organized by reminding you of upcoming tasks and keeping a record of your car's maintenance history. Everything works offline so you can check your car info even when you don't have internet.

### car entity
each car in the system has the following information:

- id: the unique identifier of the car
- make: the manufacturer of the car
- model: the specific model name
- year: the year the car was manufactured
- license plate: the car's registration number
- current mileage: total kilometers driven
- last ITP date: when the car last passed its technical inspection
- next ITP date: when the next technical inspection is due
- rovinieta expiry date: when the road tax needs to be renewed
- description: any additional information about the car (optional)

### create
- on pressing the "Add Car" button from the main screen, a new modal opens
- the modal contains input fields for: make, model, year, license plate, current mileage, last ITP date, next ITP date, rovinieta expiry date, and description
- on pressing "Save", the new car is created and stored in the local database
- the modal closes and the user returns to the car list which now includes the new car
- if online, the new car is synchronized to the server

### read
- when the application starts, it reads all cars from the local database
- the main screen displays a list of all cars with basic information (make, model, license plate)
- each car card shows visual indicators for upcoming deadlines (ITP date and rovinieta expiry in red if expired or orange if expiring soon)
- on pressing a car from the list, the app navigates to the car detail screen
- the detail screen displays all car information including all dates and description

### update
- on pressing the "Edit" button on a car detail screen, a modal opens pre-filled with current data
- all fields are editable (make, model, year, license plate, current mileage, last ITP date, next ITP date, rovinieta expiry date, description)
- on pressing "Save", the car record is updated in the local database
- the detail screen refreshes to show the updated information
- if online, changes are synchronized to the server

### delete 
- on pressing the "Delete" button on a car detail screen, a confirmation dialog appears
- the dialog asks "Are you sure you want to delete this car?"
- on confirming, the car is deleted from the local database
- the app navigates back to the main car list
- if online, the deletion is synchronized to the server


### localk db
All CRUD operations are first saved to the local database:
- create: new cars are immediately saved locally
- read: data is always read from the local database for fast access
- update: changes are saved locally first
- delete: deletions are performed on local database first

### server sync
When the device has internet connection:
- all local changes (create, update, delete) are synchronized to the server
- if conflicts occur, the most recent change wins based on the last updated date

## offline

### create when offline
- when offline, pressing "Add Car" works exactly the same as when online
- the new car is saved to the local database immediately
- the create operation is added to a sync queue
- when internet returns, the new car is automatically pushed to the server
- a small indicator shows that sync is in progress until the operation completes

### read when offline
- reading cars works normally without any internet connection
- all car data is stored locally so the list and detail screens load instantly
- users can browse all their cars, view details, and see all dates and information
- no sync needed since data is already on the device

### update when offline
- when offline, pressing "Edit" on a car detail screen works the same as online
- changes are saved to the local database right away
- the update operation is queued for synchronization
- when connection is restored, the updated data is sent to the server
- the app shows when updates are still pending sync

### delete when offline  
- when offline, pressing "Delete" removes the car from the local database
- the deletion is added to the sync queue to be processed later
- when back online, the deletion is synchronized to the server
- if the same car was modified on another device while offline, the deletion takes priority

