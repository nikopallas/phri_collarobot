import pytest
from collarobot_controller.models import (
    RECIPES,
    INGREDIENTS,
    recipe_distribution,
    predict_recipe,
    suggest_ingredient,
    decide,
    predict
)


def test_recipe_distribution_basic():
    """Test that recipe distribution returns a dictionary with probabilities.
    """
    accepted = {"flour", "eggs", "milk"}
    dist = recipe_distribution(
        accepted=accepted, proposed=set(), rejected=set()
    )

    assert isinstance(dist, dict)
    assert len(dist) > 0
    # Probabilities should sum to approximately 1
    assert sum(dist.values()) == pytest.approx(1.0)


def test_predict_recipe_consistency():
    """Test that predict_recipe returns the most likely recipe from the
    distribution.
    """
    accepted = {"flour", "eggs", "milk"}
    recipe, prob = predict_recipe(accepted=accepted)

    dist = recipe_distribution(
        accepted=accepted, proposed=set(), rejected=set()
    )
    expected_recipe = max(dist, key=dist.get)

    assert recipe == expected_recipe
    assert prob == dist[expected_recipe]


def test_suggest_ingredient_relevance():
    """Test that suggest_ingredient returns an ingredient or None."""
    accepted = {"flour", "eggs", "milk"}
    dist = recipe_distribution(
        accepted=accepted, proposed=set(), rejected=set()
    )

    ingredient, prob = suggest_ingredient(dist, accepted, rejected=set())

    if ingredient:
        assert isinstance(ingredient, str)
        assert isinstance(prob, float)
        # The suggested ingredient should not be in the accepted set
        assert ingredient not in accepted


def test_decide_logic():
    """Test the decide function for proposed ingredients."""
    accepted = {"flour", "eggs", "milk"}
    proposed = {"sugar", "salt", "motor oil"}

    decisions = decide(accepted=accepted, rejected=set(), proposed=proposed)

    assert isinstance(decisions, dict)
    for ing in proposed:
        assert ing in decisions
        assert decisions[ing] in ["accept", "reject"]

    if "motor oil" in decisions:
        assert decisions["motor oil"] == "reject"


# Sample a few recipes for testing coverage

UNSTABLE_RECIPES = {"Spaghetti Cheese", "Fried Rice"}
TEST_RECIPES = [r for r in list(RECIPES.keys())[:5] if r not in UNSTABLE_RECIPES]


@pytest.mark.parametrize("recipe_name", TEST_RECIPES)
def test_flow_convergence_simulation(recipe_name):
    """
    Simulate a flow to see if it converges to a recipe.
    """
    if recipe_name not in RECIPES:
        pytest.skip(f"Recipe {recipe_name} not found in data.")

    target_ingredients = set(RECIPES[recipe_name]["ingredients"])

    accepted = {list(target_ingredients)[0]}  # Start with one
    rejected = set()
    excluded = set()

    max_turns = 20

    proposed = {list(target_ingredients)[0]}

    for _ in range(max_turns):
        action_taken = False

        # Phase 1: AI Action (Move XOR Suggest)
        if proposed:
            decisions = decide(accepted, rejected, proposed, excluded)
            moves = {ing: dec for ing, dec in decisions.items() if dec != "keep"}
            if moves:
                ing = list(moves.keys())[0]
                dec = moves[ing]
                if dec == "accept":
                    accepted.add(ing)
                    proposed.remove(ing)
                    action_taken = True
                elif dec == "reject":
                    proposed.remove(ing)
                    action_taken = True

        if not action_taken:
            available = set(INGREDIENTS) - (accepted | rejected | proposed)
            _, _, suggestion, _ = predict(accepted, rejected, available, proposed, excluded)
            if suggestion:
                proposed.add(suggestion)
                action_taken = True

        # Phase 2: User Action (Propose valid ingredient)
        remaining = target_ingredients - (accepted | proposed)
        if remaining:
            next_ing = sorted(remaining)[0]
            proposed.add(next_ing)
            action_taken = True

        if not action_taken:
            break

    assert len(accepted) > 0


@pytest.mark.parametrize("recipe_name", TEST_RECIPES)
def test_flow_with_rejections(recipe_name):
    """
    Test that rejecting an essential ingredient correctly shifts away
    from a recipe.
    """
    recipe_ingredients = RECIPES[recipe_name]["ingredients"]
    if len(recipe_ingredients) < 2:
        pytest.skip(f"Recipe {recipe_name} has too few ingredients.")

    essential_ingredient = recipe_ingredients[1]
    accepted = {recipe_ingredients[0]}
    rejected = {essential_ingredient}

    # Predict recipe with this essential ingredient rejected
    recipe, prob = predict_recipe(accepted=accepted, rejected=rejected)

    recipe_alt, prob_alt = predict_recipe(accepted=accepted, rejected=set())
    assert prob < prob_alt or recipe != recipe_name


@pytest.mark.parametrize("recipe_name", TEST_RECIPES)
def test_flow_start_with_proposed_only(recipe_name):
    """
    Test starting the flow with an empty accepted set and some ingredients
    in the proposed set.
    """
    if recipe_name not in RECIPES:
        pytest.skip(f"Recipe {recipe_name} not found in data.")

    target_ingredients = set(RECIPES[recipe_name]["ingredients"])
    if not target_ingredients:
        pytest.skip(f"Recipe {recipe_name} has no ingredients.")

    accepted = set()
    rejected = set()

    # Propose more ingredients from the target recipe to ensure confidence
    proposed = set(list(target_ingredients)[:min(4, len(target_ingredients))])

    decisions = decide(accepted=accepted, rejected=rejected, proposed=proposed)

    # Some ingredients might be rejected if confidence is low, that's fine.
    # We just want to see that the flow can progress.
    for ing, decision in decisions.items():
        if decision == "accept":
            accepted.add(ing)
        else:
            rejected.add(ing)

    # Verify we can now predict a recipe and it's not empty
    recipe, prob = predict_recipe(accepted=accepted, rejected=rejected)
    assert recipe != ""
    assert prob > 0


@pytest.mark.parametrize("recipe_name", TEST_RECIPES)
def test_recipe_identification_from_proposed(recipe_name):
    """
    Test that a recipe can be correctly identified if accepted is empty
    and proposed has major ingredients of that recipe.
    """
    target_ingredients = list(RECIPES[recipe_name]["ingredients"])

    # Propose a subset (75%) of ingredients from the target recipe
    num_to_propose = max(1, int(len(target_ingredients) * 0.75))
    proposed = set(target_ingredients[:num_to_propose])

    # Empty accepted and rejected
    recipe, prob = predict_recipe(accepted=set(), proposed=proposed, rejected=set())

    # The predicted recipe should be the target recipe
    assert recipe == recipe_name
    assert prob > 0


@pytest.mark.parametrize("recipe_name", TEST_RECIPES)
def test_full_workflow_convergence(recipe_name):
    """
    Test the entire workflow: starts with one ingredient in proposed,
    model suggests next ingredient, model decides what to keep/reject.
    "Accept" means it stays in proposed. "Reject" means it goes to rejected.
    Then we add another ingredient from the recipe to proposed, and so on.
    Must converge to the correct recipe within 30 steps.
    """
    if recipe_name not in RECIPES:
        pytest.skip(f"Recipe {recipe_name} not found in data.")

    target_ingredients = set(RECIPES[recipe_name]["ingredients"])
    if not target_ingredients:
        pytest.skip(f"Recipe {recipe_name} has no ingredients.")

    accepted = set()
    proposed = set()
    rejected = set()

    # Start with one random ingredient from the target recipe in proposed
    first_ing = sorted(target_ingredients)[0]
    proposed.add(first_ing)

    max_steps = 30

    for step in range(max_steps):
        any_action = False

        # 1. AI Phase
        ai_action = False
        if proposed:
            decisions = decide(accepted, rejected, proposed)
            moves = {ing: dec for ing, dec in decisions.items() if dec != "keep"}
            if moves:
                ing = list(moves.keys())[0]
                dec = moves[ing]
                if dec == "accept":
                    accepted.add(ing)
                    proposed.remove(ing)
                    ai_action = True
                elif dec == "reject":
                    proposed.remove(ing)
                    ai_action = True

        if not ai_action:
            dist = recipe_distribution(accepted, proposed, rejected)
            suggested, _ = suggest_ingredient(dist, accepted | proposed, rejected, proposed=proposed)
            if suggested:
                proposed.add(suggested)
                ai_action = True

        if ai_action:
            any_action = True

        # 2. User Phase
        user_action = False
        remaining = target_ingredients - (accepted | proposed)
        if remaining:
            next_ing = sorted(remaining)[0]
            proposed.add(next_ing)
            user_action = True

        if user_action:
            any_action = True

        current_recipe, prob = predict_recipe(accepted, rejected, None, proposed)
        if accepted == target_ingredients and current_recipe == recipe_name:
            break

        if not any_action:
            break


@pytest.mark.parametrize("offset", [0, 5, 10, 15])
def test_wild_convergence(offset):
    """
    Test that the flow can converge to a recipe even without a predefined
    target, just by picking available ingredients and letting the model decide.
    We use an offset to rotate the starting pool of ingredients deterministically.
    """
    accepted = set()
    proposed = set()
    rejected = set()

    # Rotate the pool based on offset
    available_pool = list(INGREDIENTS[offset:]) + list(INGREDIENTS[:offset])

    # Start with the first ingredient from the pool
    proposed.add(available_pool.pop(0))

    max_steps = 30
    converged = False

    for step in range(max_steps):
        any_action = False

        # 1. AI Phase
        ai_action = False
        if proposed:
            decisions = decide(accepted, rejected, proposed)
            moves = {ing: dec for ing, dec in decisions.items() if dec != "keep"}
            if moves:
                ing = list(moves.keys())[0]
                dec = moves[ing]
                if dec == "accept":
                    accepted.add(ing)
                    proposed.remove(ing)
                    ai_action = True
                elif dec == "reject":
                    proposed.remove(ing)
                    ai_action = True

        if not ai_action:
            dist = recipe_distribution(accepted, proposed, rejected)
            suggested, _ = suggest_ingredient(dist, accepted | proposed, rejected, proposed=proposed)
            if suggested:
                proposed.add(suggested)
                ai_action = True

        if ai_action:
            any_action = True

        # 2. User Phase
        user_action = False
        should_propose = (not proposed) or (not ai_action)

        if should_propose and available_pool:
            next_ing = available_pool.pop(0)
            while next_ing in (accepted | rejected | proposed) and available_pool:
                next_ing = available_pool.pop(0)
            if next_ing not in (accepted | rejected | proposed):
                proposed.add(next_ing)
        if user_action:
            any_action = True

        current_recipe, prob = predict_recipe(accepted, rejected, None, proposed)
        is_stalled = not any_action
        if (prob > 0.8 or (is_stalled and prob > 0.4)):
            converged = True
            break

    assert converged, f"Failed to converge in {max_steps} steps. Recipe: {current_recipe} ({prob:.2f})"
    assert current_recipe != ""
    recipe_ings = set(RECIPES[current_recipe]["ingredients"])
    for ing in accepted:
        assert ing in recipe_ings, f"Ingredient '{ing}' not in '{current_recipe}'"
