from unittest import TestCase

from seminar10.domain.car import Car


class TestCar(TestCase):
    def test_car_license_plate(self):
        c = Car("CJ09ERT", "Toyota", "Corolla", "blue")
        self.assertEqual(c.license_plate, "CJ09ERT")

    def test_car_make(self):
        c = Car("CJ09ERT", "Toyota", "Corolla", "blue")
        self.assertEqual(c.make, "Toyota")

    def test_car_color(self):
        c = Car("CJ09ERT", "Toyota", "Corolla", "blue")
        self.assertEqual(c.color, "blue")
