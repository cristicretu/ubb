from unittest import TestCase

from seminar10.domain.client import Client


class TestCar(TestCase):
    def test_client(self):
        c = Client("1", "John")

        self.assertEqual(c.id, "1")
        self.assertEqual(c.name, "John")

    def test_client_eq(self):
        c1 = Client("1", "John")
        c2 = Client("1", "John")

        self.assertEqual(c1, c2)

    def test_client_neq(self):
        c1 = Client("1", "John")
        c2 = Client("2", "John")

        self.assertNotEqual(c1, c2)
