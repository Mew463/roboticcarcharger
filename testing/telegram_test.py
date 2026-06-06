from tesla.telegram_control import TelegramControl, MessagingReceiverWrapper
import time
import logging
t = TelegramControl()
wrapper = MessagingReceiverWrapper(t)
wrapper.start()

# Test logger functionality
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("Telegram_Test")
logger.addHandler(t)
t.send_message("hello world")
logger.critical("Critical message")
while True:
    print(wrapper.get_cmd())
    time.sleep(1)