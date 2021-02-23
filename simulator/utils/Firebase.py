import pyrebase

config = {
    "apiKey": "AIzaSyBJ40d8bNo2x_reFNjfxCUinALH24Rzh9Y",
    "authDomain": "seer-3ec9b.firebaseapp.com",
    "databaseURL": "https://seer-3ec9b-default-rtdb.firebaseio.com",
    "storageBucket": "seer-3ec9b.appspot.com"
}

# TODO: Move this to .env file
NAMESPACE = 'simulator'

firebase = pyrebase.initialize_app(config)
db = firebase.database()

# Remove any old data that might be there
db.child(NAMESPACE).child('live_report').remove()