import unittest
from domain import Taxi
from repository import TaxiRepository

from services import TaxiService


class Tests(unittest.TestCase):
    def test_taxi_ride(self):
        taxi_service = TaxiService(TaxiRepository())

        taxi_service.add_taxi(1, 0, 0, 0)
        taxi_service.add_taxi(2, 50, 50, 0)
        taxi_service.add_taxi(3, 100, 100, 0)

        print(taxi_service.get_taxis())

        taxi_service.ride(0, 0, 10, 10)

        taxis = taxi_service.sort_taxis_by_fare()

        print(taxis)

        self.assertEqual(taxis[0].id, 1)
        self.assertEqual(taxis[0].fare, 20)
        self.assertEqual(taxis[1].id, 2)
        self.assertEqual(taxis[1].fare, 0)
        self.assertEqual(taxis[0].x, 10)
        self.assertEqual(taxis[0].y, 10)