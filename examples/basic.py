import logging
import time

from six.moves import input

from create2remix import Create2, Leds

logging.basicConfig(level=logging.DEBUG)


def main():
  bot = Create2("/dev/ttyUSB0")
  bot.safe()
  bot.leds(Leds.DEBRIS, 0, 255)
  bot.digit_leds_ascii("1", "1", "1", "2")
  print("press enter to go back to stop")
  input()
  bot.drive_direct(50, 50)
  time.sleep(0.5)


if __name__ == "__main__":
  main()
