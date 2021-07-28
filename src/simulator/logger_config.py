logger_config = {
    'version': 1,
    'formatters': {
        'simple': {
            'format': '[%(levelname)s] %(asctime)s - %(name)s | %(message)s', 'datefmt': '%H:%M:%S'
        }
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'simple',
            'stream': 'ext://sys.stdout'
        }
    },
    'root': {
        'handlers': ['console']
    }
}
