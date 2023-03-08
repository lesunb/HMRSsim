import typing

Point = typing.Tuple[typing.Union[float, int], typing.Union[float, int]]
ShapeDefinition = typing.Tuple[Point, typing.List[Point]]

EVENT = typing.NamedTuple('Event', [('type', str), ('payload', typing.NamedTuple)])
ERROR = typing.NamedTuple('ErrorEvent', [('type', str), ('ent', int), ('payload', typing.NamedTuple)])

class Component:
    pass

# Payloads and tags convention related to Goto events
GotoPoiPayload = typing.NamedTuple('GotoPoiPayload', [('entity', int), ('target', str)])
GotoPosPayload = typing.NamedTuple('GotoPosPayload', [('entity', int), ('target', list)])
GotoPoiEventTag = 'GoToPoiEvent'
GotoPosEventTag = 'GoToPosEvent'

# Payloads and tags convention related to Path events
EndOfPathPayload = typing.NamedTuple('EndOfPathPayload', [('ent', int), ('timestamp', str), ('path', typing.List[Point])])
EndOfPathTag = 'EndOfPath'
EndOfApproximationPayload = typing.NamedTuple('EndOfApproximation', [('ent', int), ('timestamp', str),])
EndOfApproximationTag = 'EndOfApproximation'