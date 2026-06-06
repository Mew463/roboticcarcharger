import requests
import time
import json
import logging
import threading
from settings.config import MessagingConfig
from sensors.vision import CharucoTracking

class MessagingReceiverWrapper:
    def __init__(self, messaging):
        self.messaging = messaging
        self.last_cmd = None
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            cmd = self.messaging.process_commands()
            if cmd:
                with self.lock:
                    self.last_cmd = cmd
            time.sleep(3)  # poll rate

    def get_cmd(self):
        with self.lock:
            cmd = self.last_cmd
            self.last_cmd = None
            return cmd

class TelegramControl(logging.Handler):
    def __init__(self, stop_event, camera):
        super().__init__(logging.CRITICAL)
        self.offset = None  # track last message
        self.token = None
        self.chat_id = None
        self._load_secrets()
        self._flush_old_messages()
        self.logger = logging.getLogger(f"{__name__}")
        self.stop_event = stop_event
        self.camera = camera
    
    def _flush_old_messages(self):
        updates = self.get_updates()
        if updates:
            self.offset = updates[-1]["update_id"] + 1
        
    def _load_secrets(self):
        try:
            with open('settings/secrets.json', 'r') as f:
                secrets = json.load(f)
                self.token = secrets['TELEGRAM_BOT_TOKEN']
                self.chat_id = secrets['TELEGRAM_CHAT_ID']
        except (FileNotFoundError, KeyError) as e:
            self.logger.error(f"TeslaControl: Error loading secrets: {e}")
            raise  # Halt because we can't function without these


    def get_updates(self):
        url = f"https://api.telegram.org/bot{self.token}/getUpdates"

        params = {
            "timeout": 5,
        }

        if self.offset is not None:
            params["offset"] = self.offset

        try:
            r = requests.get(url, params=params, timeout=10)
            data = r.json()
            return data.get("result", [])
        except Exception as e:
            print("Telegram error:", e)
            return []

    def send_message(self, text):
        requests.get(
            f"https://api.telegram.org/bot{self.token}/sendMessage",
            params={
                "chat_id": self.chat_id,
                "text": text
            },
            timeout=5
        )
        
    def send_photo(self):
        self.camera.save_frame()
        url = f"https://api.telegram.org/bot{self.token}/sendPhoto"
        image_path = "sensors/img.jpg"
        with open(image_path, "rb") as photo:
            files = {
                "photo": photo
            }

            data = {
                "chat_id": self.chat_id
            }

            requests.post(
                url,
                data=data,
                files=files,
                timeout=10
            )

    def process_commands(self):
        updates = self.get_updates()

        for update in updates:
            self.offset = update["update_id"] + 1

            message = update.get("message")
            if not message:
                continue

            # only accept your chat
            if str(message["chat"]["id"]) != self.chat_id:
                continue

            text = message.get("text", "").strip().lower()

            self.logger.info(f"Received:{text}")
            
            if (text == "/help"):
                self.send_message(MessagingConfig.HELP_MESSAGE)
            elif (text == "/kill"):
                self.stop_event.set()
            elif (text == "/photo"):
                self.send_photo()
            
            return text

    def emit(self, record):
        try:
            msg = self.format(record)

            self.send_message(msg)
        except Exception:
            # Never crash logging system
            pass