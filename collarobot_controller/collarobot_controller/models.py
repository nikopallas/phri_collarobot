import json
import math
from pathlib import Path
from scipy.special import softmax
import numpy as np
from typing import Tuple

# Data paths, use the prepare_data.py script to generate them
DATA_DIR = Path(__file__).parent.parent / "data"
# DATA_DIR.mkdir(exist_ok=True)  # Avoid creating data dir if it doesn't exist where we expect it

RECIPES_PATH = DATA_DIR / "recipes.json"
INGREDIENTS_PATH = DATA_DIR / "ingredients.json"

# Weights how we value different states an ingredient has
W_ACCEPTED = 3.0
W_PROPOSED = 2.0
W_REJECTED = 6.0
W_MISSING = 0.5

with open(RECIPES_PATH) as f:
    RECIPES = json.load(f)

with open(INGREDIENTS_PATH) as f:
    INGREDIENTS = json.load(f)


# softmax over dict
def _softmax_dict(scores: dict) -> dict:
    keys = list(scores.keys())
    values = np.array(list(scores.values()), dtype=float)

    probs = softmax(values)

    return dict(zip(keys, probs))


# Probability distribution over recipies based on provided ingredients in accepted, rejected, excluded and proposed
def recipe_distribution(
        accepted: set,
        proposed: set,
        rejected: set,
        excluded: set | None = None,
) -> dict[str, float]:

    if accepted is None:
        accepted = set()
    if rejected is None:
        rejected = set()
    if proposed is None:
        proposed = set()
    if excluded is None:
        excluded = set()

    scores = {}

    for recipe, data in RECIPES.items():
        if recipe in excluded:
            continue

        recipe_ings = set(data["ingredients"])

        score = 0.0

        score += W_ACCEPTED * len(accepted & recipe_ings)
        score += W_PROPOSED * len(proposed & recipe_ings)
        score -= W_REJECTED * len(rejected & recipe_ings)
        missing = recipe_ings - (accepted | proposed)
        score -= W_MISSING * len(missing)

        scores[recipe] = score

    return _softmax_dict(scores)


# Use probability distribution to predic most likely recipy
def predict_recipe(
    accepted: set = None,
    rejected: set = None,
    available: set = None,
    proposed: set = None,
    excluded: set = None
) -> Tuple[str, float]:

    dist = recipe_distribution(accepted, proposed, rejected, excluded)

    if not dist:
        return "", 0.0

    recipe = max(dist, key=dist.get)
    return recipe, dist[recipe]


# likelihood of a specific ingredient based on the current recipie distribution.
def ingredient_likelihood(
        ingredient: str,
        recipe_dist: dict[str, float],
) -> float:

    prob = 0.0
    for recipe, r_prob in recipe_dist.items():
        if ingredient in RECIPES[recipe]["ingredients"]:
            prob += r_prob

    return prob


# Calculates a strictness parameter that resembles the confidence of the most likely recipie
def recipe_strictness(recipe_dist: dict[str, float]) -> float:
    if not recipe_dist:
        return 0.0
    return max(recipe_dist.values())


# A threshold when it doesn't make sense tho further suggest ingredients
def ingredient_stop_threshold(strictness: float) -> float:
    """
    As confidence rises, require higher ingredient relevance.
    """
    return -0.2 + 0.8 * strictness


# A suggests an ingredient based on the current state
def suggest_ingredient(
        recipe_dist: dict[str, float],
        accepted: set,
        rejected: set,
        available: set | None = None,
        proposed: set | None = None,
) -> Tuple[str | None, float]:

    if available is None:
        available = set()

    strictness = recipe_strictness(recipe_dist)
    threshold = ingredient_stop_threshold(strictness)

    scores = {}

    for ingredient in INGREDIENTS:
        if ingredient in accepted or ingredient in rejected or ingredient in (proposed or set()):
            continue

        p = sum(
            r_prob
            for recipe, r_prob in recipe_dist.items()
            if ingredient in RECIPES[recipe]["ingredients"]
        )

        off_track = 1.0 - p
        penalty = strictness * off_track
        score = p - penalty

        if ingredient in available:
            score += 0.1

        scores[ingredient] = score

    if not scores:
        return None, 1.0

    best = max(scores, key=scores.get)
    best_score = scores[best]

    if best_score < threshold:
        return None, 1.0

    return best, best_score


# Predict a Recipe and an ingredient based on the current state
def predict(
    accepted: set = None,
    rejected: set = None,
    available: set = None,
    proposed: set = None,
    excluded: set = None
) -> Tuple[str, float, str, float]:

    recipe_dist = recipe_distribution(accepted, proposed, rejected, excluded)
    recipe, recipe_prob = predict_recipe(accepted, proposed, rejected, excluded)
    ingredient, ing_prob = suggest_ingredient(recipe_dist, accepted, rejected, available, proposed)
    return recipe, recipe_prob, ingredient, ing_prob


# Returns a list of recipies we could still cook based on the current state
def feasible_recipes(
        accepted: set,
        rejected: set,
        excluded: set | None = None,
) -> set[str]:

    if excluded is None:
        excluded = set()

    feasible = set()

    for recipe, data in RECIPES.items():
        if recipe in excluded:
            continue

        recipe_ings = set(data["ingredients"])

        if not accepted.issubset(recipe_ings):
            continue

        if rejected & recipe_ings:
            continue

        feasible.add(recipe)

    return feasible


def normalized_entropy(dist: dict[str, float]) -> float:
    """
    Calculates Shannon entropy normalized by max possible entropy (log N).
    Result is between 0.0 (certainty) and 1.0 (pure uncertainty).
    """
    if not dist:
        return 0.0

    vals = [v for v in dist.values() if v > 0]
    if len(vals) <= 1:
        return 0.0

    entropy = -sum(p * math.log(p) for p in vals)
    max_entropy = math.log(len(dist))
    return entropy / max_entropy


# Decides which ingredients in proposed are accepted or rejected
def decide(
        accepted: set,
        rejected: set,
        proposed: set,
        excluded: set | None = None,
) -> dict[str, str]:
    """
    Decides for each ingredient in proposed if it should be accepted, rejected
    or kept in proposed. Returns a dict mapping ingredient to decision.
    Decisions: "accept", "reject", "keep"
    """
    recipe_dist = recipe_distribution(
        accepted=accepted,
        rejected=rejected,
        proposed=proposed,
        excluded=excluded,
    )

    if not recipe_dist:
        return {ing: "reject" for ing in proposed}

    entropy = normalized_entropy(recipe_dist)
    decisions = {}

    for ing in sorted(proposed):
        likelihood = ingredient_likelihood(ing, recipe_dist)

        # Entropy-based competition
        score_accept = likelihood
        score_reject = 1 - likelihood
        score_keep = entropy

        if score_accept >= score_reject and score_accept >= score_keep:
            decision = "accept"
        elif score_reject >= score_keep:
            decision = "reject"
        else:
            decision = "keep"

        # veto decision if it kills all recipes (only for accept/keep)
        if decision != "reject":
            test_accepted = accepted.copy()
            test_rejected = rejected.copy()

            if decision == "accept":
                test_accepted.add(ing)

            feasible = feasible_recipes(
                accepted=test_accepted,
                rejected=test_rejected,
                excluded=excluded,
            )

            if not feasible:
                decision = "reject"

        decisions[ing] = decision

    return decisions
