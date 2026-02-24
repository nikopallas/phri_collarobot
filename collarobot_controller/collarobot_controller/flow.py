import json
from pathlib import Path
from models import pick_action
import random

DATA_DIR = Path(__file__).parent.parent / "data"
# DATA_DIR.mkdir(exist_ok=True)

# RECIPE = "Spaghetti Bolognese"
RECIPE = "Savory Pancakes"

INGREDIENTS_PATH = DATA_DIR / "ingredients.json"

RECIPES_PATH = DATA_DIR / "recipes.json"
with open(INGREDIENTS_PATH) as f:
    INGREDIENTS = list(json.load(f).keys())

with open(RECIPES_PATH) as f:
    RECIPES = json.load(f)

accepted = set()
rejected = set()
excluded = set()
ingredients = set(RECIPES[RECIPE].get("ingredients"))
proposed = {list(ingredients)[0]}

index = 0
seed = 104

random.seed(seed)

# Simulated user data
# "Non-fitting" ingredients are real ingredients from the dataset that are NOT in the target recipe
non_fitting_pool = [ing for ing in INGREDIENTS if ing not in ingredients]

while True:
    print(f"Turn {index}")
    print(f"Accepted: {sorted(list(accepted))}")
    print(f"Rejected: {sorted(list(rejected))}")
    print(f"Proposed: {sorted(list(proposed))}")
    print(f"Excluded: {sorted(list(excluded))}")

    any_action_taken = False

    # --- PHASE 1: AI ACTION (Move XOR Suggest) ---
    available = set(INGREDIENTS) - (accepted | rejected | proposed)
    action, ingredient = pick_action(accepted, proposed, available, rejected, excluded)
    
    if action != "skip":
        any_action_taken = True
        if action == "rejected":
            rejected.add(ingredient)
        # Note: action == "accepted" or "proposed" already modified accepted/proposed in models.py

    # --- PHASE 2: USER ACTION (Move XOR Propose) ---
    user_action_taken = False

    # 2a. User Move: User manually moves an ingredient (override)
    if proposed and random.random() < 0.2:
        valid_in_proposed = [p for p in proposed if p in ingredients]
        invalid_in_proposed = [p for p in proposed if p not in ingredients]

        if valid_in_proposed and random.random() < 0.6:
            user_move = random.choice(valid_in_proposed)
            accepted.add(user_move)
            proposed.remove(user_move)
            user_action_taken = True
            print(f"USER PHASE: User manually accepted {user_move}")
        elif invalid_in_proposed:
            user_move = random.choice(invalid_in_proposed)
            proposed.remove(user_move)
            user_action_taken = True
            print(f"USER PHASE: User manually rejected {user_move}")

    # 2b. User Propose: If no user move, user adds an ingredient
    if not user_action_taken and random.random() < 0.4:
        if random.random() < 0.2:
            bad_ing = random.choice(non_fitting_pool)
            if bad_ing not in proposed and bad_ing not in rejected and bad_ing not in accepted:
                proposed.add(bad_ing)
                user_action_taken = True
                print(f"USER PHASE: User proposed non-fitting ingredient {bad_ing}")
        else:
            remaining = ingredients - (accepted | rejected | proposed)
            if remaining:
                new_ing = random.choice(list(remaining))
                proposed.add(new_ing)
                user_action_taken = True
                print(f"USER PHASE: User proposed valid ingredient {new_ing}")

    if user_action_taken:
        any_action_taken = True

    if not any_action_taken:
        print("Converged: No more actions or suggestions from AI or User.")
        break

    print("-" * 20)
    index += 1

print(
    "\n\n\nFinal Recipe:\n" +
    "----------------\n" +
    RECIPE +
    "\n----------------\n" +
    "Ingredients: \n\n" +
    f"{'\n'.join([ingredient.strip() for ingredient in RECIPES[RECIPE].get('ingredients')])}" +
    "\n----------------\nInstructions:\n\n" +
    f"{'\n'.join([instruction.strip() for instruction in RECIPES[RECIPE].get('instructions')])}\n----------------\n")
