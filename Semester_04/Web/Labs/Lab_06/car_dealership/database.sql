CREATE DATABASE IF NOT EXISTS car_dealership;
USE car_dealership;

CREATE TABLE IF NOT EXISTS categories (
  id INT NOT NULL AUTO_INCREMENT,
  name VARCHAR(100) NOT NULL,
  description TEXT,
  PRIMARY KEY (id)
);

CREATE TABLE IF NOT EXISTS cars (
  id INT NOT NULL AUTO_INCREMENT,
  model VARCHAR(100) NOT NULL,
  engine_power VARCHAR(50) NOT NULL,
  fuel_type VARCHAR(50) NOT NULL,
  price DECIMAL(10,2) NOT NULL,
  color VARCHAR(50) NOT NULL,
  year INT NOT NULL,
  history TEXT,
  category_id INT NOT NULL,
  created_at DATETIME NOT NULL,
  PRIMARY KEY (id),
  FOREIGN KEY (category_id) REFERENCES categories(id)
);

INSERT INTO categories (name, description) VALUES
('Sedan', 'Four-door passenger cars'),
('SUV', 'Sport Utility Vehicles');

INSERT INTO cars (model, engine_power, fuel_type, price, color, year, history, category_id, created_at) VALUES
('Toyota Camry', '203 hp', 'Gasoline', 15000.00, 'Silver', 2018, 'Single owner', 1, NOW()),
('Honda CR-V', '190 hp', 'Gasoline', 18500.00, 'Blue', 2019, 'Two owners', 2, NOW()); 

alter table cars add column features TEXT;