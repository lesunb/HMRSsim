from typing import List
import pyrebase

demo_config = {
    "apiKey": "AIzaSyBJ40d8bNo2x_reFNjfxCUinALH24Rzh9Y",
    "authDomain": "seer-3ec9b.firebaseapp.com",
    "databaseURL": "https://seer-3ec9b-default-rtdb.firebaseio.com",
    "storageBucket": "seer-3ec9b.appspot.com"
}

class Firebase_conn():
    def __init__(self, namespace: str, config=demo_config):
        self.firebase = pyrebase.initialize_app(config)
        self.__db = self.firebase.database()
        self.namespace = namespace

    def clean_old_simulation(self):
        """Clean previous simulation from firebase"""
        self.__db.child(self.namespace).child('live_report').remove()
        self.__db.child(self.namespace).child('logs').remove()
        return True

    def seer_consumer(self, message, msg_idx):
        """Sends Seer messages to firebase namespace"""
        if msg_idx >= 0:
            if msg_idx == 1:
                for idx, j in enumerate(message):
                    self.__db.child(self.namespace).child('live_report').child(msg_idx).child(idx).set({j: message[j]})
            else:
                self.__db.child(self.namespace).child('live_report').child(msg_idx).set(message)

    def send_build_report(self, build_report: List[str]):
        """Sends the simulator build report to firebase"""
        self.__db.child(self.namespace).child('logs').set(build_report)
