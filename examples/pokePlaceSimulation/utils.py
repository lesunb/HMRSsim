
from examples.pokePlaceSimulation.components.Recipe import Recipe


def recipe_from_json(json: dict, name: str) -> Recipe:
    prep_time = json['prep_time']
    ingredients = json['ingredients']
    stove = json['usesStove']
    return Recipe(name, prep_time, ingredients, stove)
