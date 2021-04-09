import pyrebase

config = {
    "apiKey": "AIzaSyBJ40d8bNo2x_reFNjfxCUinALH24Rzh9Y",
    "authDomain": "seer-3ec9b.firebaseapp.com",
    "databaseURL": "https://seer-3ec9b-default-rtdb.firebaseio.com",
    "storageBucket": "seer-3ec9b.appspot.com"
}


firebase = pyrebase.initialize_app(config)
db = firebase.database()


def clean_old_simulation(namespace: str):
    """Clean previous simulation from firebase"""
    db.child(namespace).child('live_report').remove()
    db.child(namespace).child('logs').remove()
