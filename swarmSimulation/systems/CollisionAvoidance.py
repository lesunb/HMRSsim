from simulator.components.ProximitySensor import ProximitySensor


def dont(sensor: ProximitySensor):
    while True:
        ev = yield sensor.reply_channel.get()
        payload = ev.payload
        me = payload.ent
        they = payload.close_entities