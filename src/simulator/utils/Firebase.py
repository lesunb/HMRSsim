from typing import List
import pyrebase

config = {
    "apiKey": "AIzaSyBJ40d8bNo2x_reFNjfxCUinALH24Rzh9Y",
    "authDomain": "seer-3ec9b.firebaseapp.com",
    "databaseURL": "https://seer-3ec9b-default-rtdb.firebaseio.com",
    "storageBucket": "seer-3ec9b.appspot.com"
}


firebase = pyrebase.initialize_app(config)
__db = firebase.database()


def clean_old_simulation(namespace: str):
    """Clean previous simulation from firebase"""
    __db.child(namespace).child('live_report').remove()
    __db.child(namespace).child('logs').remove()

def create_consumer_for_namespace(namespace: str):
    """Returns a consumer function binded to the namespace"""
    clean_old_simulation(namespace)

    def firebase_seer_consumer(message, msg_idx):
        """Sends Seer messages to firebase namespace"""
        if msg_idx >= 0:
            if msg_idx == 1:
                for idx, j in enumerate(message):
                    __db.child(namespace).child('live_report').child(msg_idx).child(idx).set({j: message[j]})
            else:
                __db.child(namespace).child('live_report').child(msg_idx).set(message)
    
    return firebase_seer_consumer

def send_build_report(namespace: str, build_report: List[str]):
    """Sends the simulator build report to firebase"""
    __db.child(namespace).child('logs').set(build_report)
