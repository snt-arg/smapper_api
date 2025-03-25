import unittest

from fastapi.testclient import TestClient
from app.main import app


class TestAPIEndpoints(unittest.TestCase):
    api_version = "v1"
    api_url = f"http://127.0.0.1:8000/api/{api_version}"

    def setUp(self) -> None:
        self.client = TestClient(app, self.api_url)

    def test_get_sensors(self):
        resp = self.client.get("/sensors")

        self.assertEqual(resp.status_code, 200)

    def test_get_services(self):
        resp = self.client.get("/services")

        self.assertEqual(resp.status_code, 200)

    def test_get_bags(self):
        resp = self.client.get("/bags")
        self.assertEqual(resp.status_code, 200)
