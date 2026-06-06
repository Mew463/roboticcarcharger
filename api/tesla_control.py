import requests
import json
import logging 
import math
from settings.config import CarConfig

class TeslaControl:
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}")
        self.token = None
        self.vin = None
        self.car_data = None
        self.charger_location = None
        self._load_secrets()

    def _load_secrets(self):
        try:
            with open('settings/secrets.json', 'r') as f:
                secrets = json.load(f)
                self.token = secrets['TESSIE_API_TOKEN']
                self.vin = secrets['TESLA_VIN']
                self.charger_location = secrets['CHARGER_LOCATION']
        except (FileNotFoundError, KeyError) as e:
            self.logger.error(f"TeslaControl: Error loading secrets: {e}")
            raise  # Halt because we can't function without these

    def _send_command(self, endpoint, params=None):
        """Helper function to handle all POST requests to Tessie."""
        if not self.token or not self.vin:
            self.logger.error("TeslaControl: Missing credentials.")
            return False

        url = f"https://api.tessie.com/{self.vin}/command/{endpoint}"
        
        # Default parameters for all commands
        default_params = {
            "wait_for_completion": "true",
            "max_attempts": 3
        }
        if params:
            default_params.update(params)

        headers = {
            "Authorization": f"Bearer {self.token}",
            "Content-Type": "application/json"
        }

        try:
            self.logger.info(f"TeslaControl: Sending command '{endpoint}'...")
            response = requests.post(url, headers=headers, params=default_params, timeout=30)
            response.raise_for_status()
            
            result = response.json()
            if result.get('result') is True:
                self.logger.info(f"TeslaControl: '{endpoint}' successful.")
                return True
            else:
                self.logger.warning(f"TeslaControl: '{endpoint}' failed. Response: {result}")
                return False

        except requests.exceptions.RequestException as e:
            self.logger.error(f"TeslaControl: API Error during '{endpoint}': {e}")
            return False
        
    def _get_car_data(self) -> bool | None:
        """Returns the current data of the vehicle."""
        url = f"https://api.tessie.com/{self.vin}/state"
        params = {"use_cache": "true"}
        headers = {"Authorization": f"Bearer {self.token}"}

        try:
            response = requests.get(url, headers=headers, params=params)
            response.raise_for_status()
            
            self.car_data = response.json()
            return True
        
        except Exception as e:
            self.logger.warning(f"TeslaControl: Failed to get status: {e}")
            return None
        
    # --- Public Get Commands ---
    def get_charging_related_data(self):
        self._get_car_data()
        car_data = self.car_data
        if (car_data is None):
            return None
        else:
            return_dict = {
                "charge_state" : car_data.get('charge_state', {}).get('charging_state'),
                "soc" : car_data.get('charge_state', {}).get('battery_level'),
                "charge_limit" : car_data.get('charge_state', {}).get('charge_limit_soc'),
                "shift_state" : car_data.get('drive_state', {}).get('shift_state'),
                "is_at_charger" : self.is_at_charger(car_data)
            }
            return return_dict
        
    def car_ready_to_be_charged(self) -> bool:
        car_state = self.get_charging_related_data()
        if (car_state is not None):
            self.logger.debug(f"car_state: {car_state}")
            return (car_state["charge_state"] == "Disconnected" and 
                            car_state["is_at_charger"] and 
                            car_state["shift_state"] == 'P')
        else:
            self.logger.warning("car_state is None")  
            return False
        
        
    def get_charging_status(self) -> str | None:
        if (self._get_car_data() is None):
            return None
        charge_status = self.car_data.get('charge_state', {}).get('charging_state')
        return charge_status  
        
    def get_SOC(self) -> str | None:
        if (self._get_car_data() is None):
            return None
        soc = self.car_data.get('charge_state', {}).get('battery_level')
        return soc  
        
    def get_charge_limit(self) -> str | None:
        if (self._get_car_data() is not None):
            return self.car_data.get('charge_state', {}).get('charge_limit_soc')
        else:
            return None
        
    def get_shift_state(self) -> str | None:
        if (self._get_car_data() is not None):
            return self.car_data.get('drive_state', {}).get('shift_state')
        else:
            return None

    def is_at_charger(self, car_data):
        lat1 = car_data.get('drive_state', {}).get('latitude')
        lon1 = car_data.get('drive_state', {}).get('longitude')
        
        R = 6371000  # Earth radius in meters
        lat2 = self.charger_location[0]
        lon2 = self.charger_location[1]
        radius_m = CarConfig.GEOFENCE_RADIUS_METERS
            
        # convert degrees → radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c

        return distance <= radius_m

    # COMMANDS TO VEHICLE

    def toggle_trunk(self):
        """Activates the rear trunk."""
        return self._send_command("activate_rear_trunk")

    def open_or_unlatch_charge_port(self, num_tries = 3):
        # return self._send_command("open_charge_port")
        """Opens or unlatches the charge port."""
        success = False
        for i in range (num_tries):
            success = self._send_command("open_charge_port")
            if (success):
                return True
        return False

    def stop_charging(self):
        """Stops the charging session."""
        return self._send_command("stop_charging")