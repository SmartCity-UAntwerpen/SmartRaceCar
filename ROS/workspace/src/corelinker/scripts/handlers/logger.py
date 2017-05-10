import logging


def init_logger():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')


def log_debug(text):
    logging.debug(text)


def log_info(text):
    logging.info(text)


def log_warning(text):
    logging.warning(text)


def log_critical(text):
    logging.critical(text)
