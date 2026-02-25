from collarobot_controller.models import pick_action, INGREDIENTS


def test_pick_action_suggestion():
    accepted = set()
    proposed = set()
    rejected = set()
    excluded = set()
    available = set(INGREDIENTS)

    # Initially, no ingredients accepted/proposed. Should suggest something.
    action, ingredient = pick_action(accepted, proposed, available, rejected, excluded)

    assert action == "proposed"
    assert ingredient is not None
    assert ingredient in proposed


def test_pick_action_decision():
    # Scenario: "Savory Pancakes"
    # Ingredients: "flour", "eggs", "milk", "salt", "green onions"
    accepted = {"flour", "eggs"}
    proposed = {"milk"}
    rejected = set()
    excluded = set()
    available = set(INGREDIENTS) - (accepted | proposed)

    # AI should likely accept "milk" for Savory Pancakes
    action, ingredient = pick_action(accepted, proposed, available, rejected, excluded)

    assert action in ["accepted", "proposed", "skip"]
    if action == "accepted":
        assert ingredient == "milk"
        assert "milk" in accepted
        assert "milk" not in proposed


def test_pick_action_rejection():
    # Scenario: "Savory Pancakes" but user proposes "chocolate"
    accepted = {"flour", "eggs", "milk"}
    proposed = {"chocolate"}
    rejected = set()
    excluded = set()
    available = set(INGREDIENTS) - (accepted | proposed)

    action, ingredient = pick_action(accepted, proposed, available, rejected, excluded)

    if action == "rejected":
        assert ingredient == "chocolate"
        # Since we removed rejected.add(ing) from models.py, it shouldn't be in rejected yet
        assert "chocolate" not in rejected
        assert "chocolate" not in proposed
