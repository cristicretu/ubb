from seminar10.domain.car import Car


import datetime
from unittest import TestCase

from seminar10.domain.client import Client
from seminar10.domain.rental import Rental


class TestRental(TestCase):
    def test_rental(self):
        r1 = Rental(
            "1",
            Client("1", "John"),
            Car("1", "Ford", "Focus", "blue"),
            datetime.date(2019, 10, 10),
            datetime.date(2019, 10, 20),
        )

        self.assertEqual(r1.id, "1")
        self.assertEqual(r1.client, Client("1", "John"))

    def test_rental_eq(self):
        r1 = Rental(
            "1",
            Client("1", "John"),
            Car("1", "Ford", "Focus", "blue"),
            datetime.date(2021, 1, 1),
            datetime.date(2021, 1, 1),
        )
        r2 = Rental(
            "1",
            Client("1", "John"),
            Car("1", "Ford", "Focus", "blue"),
            datetime.date(2019, 10, 10),
            datetime.date(2019, 10, 20),
        )

        self.assertNotEqual(r1, r2)
